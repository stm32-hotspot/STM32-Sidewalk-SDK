#
##############################################################################
# file:    deploy_geolocation_demo_stack.py
# brief:   Deploys all the necessary cloud components to receive and process
#          geolocation demo messages from end nodes
##############################################################################
#
# Copyright (c) 2025 STMicroelectronics.
# All rights reserved.
#
# This software is licensed under terms that can be found in the LICENSE file
# in the root directory of this software component.
# If no LICENSE file comes with this software, it is provided AS-IS.
#
##############################################################################
#
import subprocess
import sys

from constants.geolocation_demo_constants import GeolocationDemoConstants

from helpers.boto_session import BotoSession
from helpers.cloudformation_template_handler import CloudFormationTemplateHandler

from libs.cloud_formation_client import CloudFormationClient
from libs.config import Config
from libs.iot_wireless_client import IoTWirelessClient
from libs.s3_client import S3Client
from libs.utils import *


def build_device_provisioning_data(device_data: dict, device_profile_data: dict) -> dict:
    device_provisioning_data = {}

    # Populate device certificates
    for cert in device_data['Sidewalk']['DeviceCertificates']:
        if cert['SigningAlg'] == 'P256r1':
            device_provisioning_data['p256R1'] = cert['Value']
        elif cert['SigningAlg'] == 'Ed25519':
            device_provisioning_data['eD25519'] = cert['Value']

    # Store app server public key
    device_provisioning_data['applicationServerPublicKey'] = device_profile_data['Sidewalk']['ApplicationServerPublicKey']

    # Populate device metadata
    device_provisioning_data['metadata'] = {
        'applicationDeviceArn': device_data['Arn'],
        'applicationDeviceId': device_data['Id'],
        'smsn': device_data['Sidewalk']['SidewalkManufacturingSn']
    }

    for profile_cert_metadata in device_profile_data['Sidewalk']['DakCertificateMetadata']:
        if profile_cert_metadata['CertificateId'] == device_data['Sidewalk']['CertificateId']:
            device_provisioning_data['metadata']['deviceTypeId'] = profile_cert_metadata['DeviceTypeId']

    for private_key in device_data['Sidewalk']['PrivateKeys']:
        if private_key['SigningAlg'] == 'P256r1':
            device_provisioning_data['metadata']['devicePrivKeyP256R1'] = private_key['Value']
        elif private_key['SigningAlg'] == 'Ed25519':
            device_provisioning_data['metadata']['devicePrivKeyEd25519'] = private_key['Value']

    return device_provisioning_data


def main():
    try:
        # -----------------
        # Read config file
        # -----------------
        config = Config()

        # --------------------
        # Ask user to proceed
        # --------------------
        log_info('Arguments to be used during the Sidewalk Geolocation Demo deployment:')
        if config.aws_profile: log_info(f'\tCONFIG_PROFILE: {config.aws_profile}')
        log_info(f'\tREGION: {config.region_name}')
        log_info(f'\tSID_DEMO_NAME: {config.sid_demo_name}')
        if config.interactive_mode:
            log_info(f'Proceed with stack creation?')
            confirm()
        log_info(f'Deploying the app to AWS. This can take several minutes to complete')

        # -------------------------------------------------------------
        # Build constants based on the AWS account and credentials
        # -------------------------------------------------------------
        constants = GeolocationDemoConstants(config)

        # -------------------------------------------------------------
        # Create boto3 session using given profile and service clients
        # Sidewalk is only enabled in the us-east-1 region
        # -------------------------------------------------------------
        boto_session = BotoSession(config)

        cf_client = CloudFormationClient(boto_session)
        s3_client = S3Client(boto_session)
        wireless_client = IoTWirelessClient(boto_session)

        # ---------------------------------------------------
        # Check if given Sidewalk destination already exists
        # ---------------------------------------------------
        sid_dest_already_exists = wireless_client.check_if_destination_exists(name=constants.SIDEWALK_DESTINATION_NAME)
        if sid_dest_already_exists:
            sid_dest_managed_by_current_stack = cf_client.check_if_resource_is_managed_by_stack(constants.SIDEWALK_DEMO_STACK_NAME, 'IoTWirelessDestination00GeolocationAppDestination')
            if not sid_dest_managed_by_current_stack:
                log_warn(f'Sidewalk destination {constants.SIDEWALK_DESTINATION_NAME} already exists and will not be included in the {constants.SIDEWALK_DEMO_STACK_NAME}')
            else:
                log_info(f'Sidewalk destination {constants.SIDEWALK_DESTINATION_NAME} exist, but it is manged by the current CloudFormation stack. It will be updated as a part of the deployment')
                sid_dest_already_exists = False
        else:
            log_success(f'Sidewalk destination {constants.SIDEWALK_DESTINATION_NAME} does not exist and will be included in the {constants.SIDEWALK_DEMO_STACK_NAME}')

        # --------------------------------------
        # Prepare parameters for CloudFormation template
        # --------------------------------------
        cf_template_parameters = {
            'SidewalkDemoName': config.sid_demo_name,
            'SidewalkGeolocationMqttRootTopic': config.sid_demo_mqtt_topi_root,
            'SidewalkGeolocationTemplateS3Bucket': constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME,
            'SidewalkGeolocationDestinationName': constants.SIDEWALK_DESTINATION_NAME,
            'SidewalkGeolocationDestinationRoleName': constants.SIDEWALK_DESTINATION_ROLE,
            'SidewalkGeolocationDestinationAlreadyExists': sid_dest_already_exists,
        }
        cf_lambda_sources = {
            'SidewalkGeolocationUplink': 'sidewalk_geolocation_uplink',
        }

        # ------------------------------------------------------------------------------
        # Create a bucket and upload CloudFormation template, because it's too large to
        # pass it via `template` arg to `cloudformation.create_stack`
        # ------------------------------------------------------------------------------
        log_info('Uploading CloudFormation template...')
        CloudFormationTemplateHandler.upload_template(s3_client=s3_client, lambda_sources=cf_lambda_sources)
        log_success('Uploaded CloudFormation template')

        # --------------------------------------
        # Trigger CloudFormation stack creation
        # --------------------------------------
        cf_client.create_stack(
            stack_name=constants.SIDEWALK_DEMO_STACK_NAME,
            lambda_sources=cf_lambda_sources,
            template_params=cf_template_parameters,
            tag=constants.SIDEWALK_DEMO_TAG,
            template=None,
            template_url=constants.CLOUDFORMATION_TEMPLATE_BUCKET_URL,
        )

        # ------------------------------------------------------------------------
        # Update given Sidewalk destination (only if destination already existed)
        # ------------------------------------------------------------------------
        if sid_dest_already_exists:
            wireless_client.update_existing_destination(
                dest_name=constants.SIDEWALK_DESTINATION_NAME,
                role_name=constants.SIDEWALK_DESTINATION_ROLE,
                interactive_mode=config.interactive_mode
            )

        # --------------------------------------
        # Create a dedicated Sidewalk device profile for this demo
        # --------------------------------------
        if wireless_client.check_if_sidewalk_device_profile_exists(constants.SIDEWALK_DEVICE_PROFILE):
            log_warn(f'Sidewalk device profile {constants.SIDEWALK_DEVICE_PROFILE} already exists, its creation is skipped')
        else:
            wireless_client.create_sidewalk_device_profile(constants.SIDEWALK_DEVICE_PROFILE)

        # --------------------------------------
        # Create a Sidewalk device and provision it
        # --------------------------------------
        if config.manage_device_instance:
            # Check if device exists already
            device =  wireless_client.get_sidewalk_device(config.sid_demo_device_name, constants.SIDEWALK_DEVICE_PROFILE)

            if device is None:
                # Create the device
                device = wireless_client.create_sidewalk_device(config.sid_demo_device_name, constants.SIDEWALK_DEVICE_PROFILE, constants.SIDEWALK_DESTINATION_NAME)
            else:
                if config.interactive_mode:
                    log_warn(f'Sidewalk device {config.sid_demo_name} exists already. Override its configuration?')
                    confirm()
                else:
                    log_warn(f'Sidewalk device {config.sid_demo_name} exists already. Its configuration will be overwritten')

                # Change device destination to the one created by this script
                wireless_client.set_sidewalk_device_destination(
                    device_name=device['Name'],
                    device_id=device['Id'],
                    destination_name=constants.SIDEWALK_DESTINATION_NAME)

            # Store device and device profile information
            device_profile = wireless_client.get_sidewalk_device_profile(constants.SIDEWALK_DEVICE_PROFILE)
            config.set_wireless_device_id(device['Id'])
            config.set_wireless_device_profile_id(device_profile['Id'])

            # Generate manufacturing data
            provisioning_data = build_device_provisioning_data(device, device_profile)
            provisioning_file_name = f'{device["Name"]}.json'
            config.store_device_provisioning_json(device_provisioning_data=provisioning_data, file_name=provisioning_file_name)
            log_success(f'Exported device provisioning information to {provisioning_file_name}')

            log_info('Running Sidewalk Provisioning Tool to generate manufacturing data...')
            ret = subprocess.call(['python', os.path.join(config.provisioning_tool_dir, 'provision.py'),
                                   'st', 'aws',
                                   f'--certificate_json={os.path.join(config.provisioning_output_dir, provisioning_file_name)}',
                                   # TODO: specify chip ID when support is added, e.g.: '--chip=stm32wba5x',
                                   f'--output_hex={os.path.join(config.provisioning_output_dir, device["Name"]+".hex")}',
                                   f'--output_bin={os.path.join(config.provisioning_output_dir, device["Name"]+".bin")}',
                                  ])
            if ret == 0:
                log_success(f'Generated manufacturing data flashing images for {device["Name"]}')
            else:
                terminate(f'Failed to generate manufacturing data flashing images for {device["Name"]}. Provisioning tool exit code: {ret}', ErrCode.EXCEPTION)

        log_success('---------------------------------------------------------------')
        log_success('Geolocation Demo has been successfully deployed to your AWS account')
        log_success('---------------------------------------------------------------')

    except Exception as e:
        print(f'Failed to deploy Geolocation Demo. Error {e}')

if __name__ == "__main__":
    sys.exit(main())
