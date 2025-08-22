#
##############################################################################
# file:    delete_geolocation_demo_stack.py
# brief:   Delete all the AWS resources created for geolocation demo
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

import sys

from constants.geolocation_demo_constants import GeolocationDemoConstants

from helpers.boto_session import BotoSession
from helpers.cloudformation_template_handler import CloudFormationTemplateHandler

from libs.cloud_formation_client import CloudFormationClient
from libs.config import Config
from libs.iot_wireless_client import IoTWirelessClient
from libs.s3_client import S3Client
from libs.utils import *


def main():
    try:
        # -----------------
        # Read config file
        # -----------------
        config = Config()

        # --------------------
        # Ask user to proceed
        # --------------------
        log_info('Arguments to be used during the Sidewalk Geolocation Demo deletion:')
        log_info(f'\tCONFIG_PROFILE: {config.aws_profile}')
        log_info(f'\tREGION: {config.region_name}')
        if config.interactive_mode:
            log_warn('This is a destructive action and can not be undone!')
            confirm()
        log_info(f'Deleting the app from AWS. This can take several minutes to complete')

        # -------------------------------------------------------------
        # Build constants based on the AWS account and credentials
        # -------------------------------------------------------------
        constants = GeolocationDemoConstants(config)

        # --------------------------------------
        # Prepare parameters for CloudFormation template
        # --------------------------------------
        cf_lambda_sources = {
            'SidewalkGeolocationUplink': 'sidewalk_geolocation_uplink',
        }

        # -------------------------------------------------------------
        # Create boto3 session using given profile and service clients
        # -------------------------------------------------------------
        boto_session = BotoSession(config)

        cf_client = CloudFormationClient(boto_session)
        s3_client = S3Client(boto_session)
        wireless_client = IoTWirelessClient(boto_session)

        # --------------------------------------
        # Delete Sidewalk device associated with this demo
        # --------------------------------------
        if config.manage_device_instance:
            # Check if device exists
            device = wireless_client.get_sidewalk_device(config.sid_demo_device_name, constants.SIDEWALK_DEVICE_PROFILE)

            if device is not None:
                # Delete the device
                wireless_client.delete_sidewalk_device(device_name=device['Name'], device_id=device['Id'])

        # -------------------------
        # Ensure no device is using IoT Core destination associated with this demo
        # -------------------------
        if wireless_client.check_if_destination_in_use(constants.SIDEWALK_DESTINATION_NAME):
            terminate('Unable to delete stack because IoT Core destination is still in use. Please reassign the '
                      'associated devices to another destination or delete them manually', ErrCode.EXCEPTION)

        # --------------------------------------
        # Cleanup the CloudFormation bucket
        # --------------------------------------
        log_info('Deleting CloudFormation template')
        CloudFormationTemplateHandler.delete_template(s3_client=s3_client, lambda_src_names=list(cf_lambda_sources.values()))
        log_success('Deleted CloudFormation template')

        # --------------------------------------
        # Delete CloudFormation stack
        # --------------------------------------
        cf_client.delete_stack(stack_name=constants.SIDEWALK_DEMO_STACK_NAME)

        # -------------------------
        # Clean up Sidewalk device profile
        # -------------------------
        if wireless_client.check_if_device_profile_in_use(constants.SIDEWALK_DEVICE_PROFILE):
            # Print a warning but don't terminate. Device profile cannot be changed after device creation, so we can't
            # do anything about that except asking the user to manually delete the devices that use our profile
            log_warn(f'Sidewalk device profile {constants.SIDEWALK_DEVICE_PROFILE} fsk_reg_profile is still in use')
        else:
            log_info(f'Sidewalk device profile {constants.SIDEWALK_DEVICE_PROFILE} is not in use and can be deleted')
            wireless_client.delete_sidewalk_device_profile(constants.SIDEWALK_DEVICE_PROFILE)

        # -------------------------
        # Clean up device provisioning files
        # -------------------------
        log_info('Removing obsolete device provisioning data...')
        for (root, dirs, files) in os.walk(config.provisioning_output_dir):
            for file in files:
                if file == f'{config.sid_demo_device_name}.json' or file == f'{config.sid_demo_device_name}.hex'  or file == f'{config.sid_demo_device_name}.bin':
                    os.remove(os.path.join(root, file))
                    log_success(f'Deleted {file}')

        # -------------------------
        # Print success message
        # -------------------------
        log_success('---------------------------------------------------------------')
        log_success('Sidewalk Geolocation Demo has been deleted')
        log_success('---------------------------------------------------------------')

    except Exception as e:
        print(f'Failed to delete Geolocation Demo. Error {e}')


if __name__ == "__main__":
    sys.exit(main())
