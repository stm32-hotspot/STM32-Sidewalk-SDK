# Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
##############################################################################
# file:    iot_wireless_client.py
# brief:   Helper class to interact with IoT Wireless service
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

import boto3
from botocore.exceptions import ClientError

from constants.geolocation_demo_constants import GeolocationDemoConstants
from libs.config import Config
from libs.utils import *


class IoTWirelessClient:
    """
    Provides IoTWirelessClient client with methods necessary handle Sidewalk destination and wireless events configuration.

    Attributes
    ----------
        _wireless_client: botocore.client.IoTWireless
            Client to IoTWireless.
        _iam_client: botocore.client.IAM
            Client to AWS IAM.

    """

    def __init__(self, session: boto3.Session):
        self._wireless_client = session.client(service_name='iotwireless')
        self._iam_client = session.client(service_name='iam')
        self._constants = GeolocationDemoConstants()
        self._config = Config()

    # -------
    # Deploy
    # -------
    def check_if_destination_exists(self, name: str) -> bool:
        """
        Checks if given destination already exists.

        :param name:    Destination name.
        :return:        True if exists, False otherwise.
        """
        try:
            log_info(f'Checking if {name} destination exists...')
            response = self._wireless_client.get_destination(Name=name)
            if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                return True
            else:
                raise Exception(f'Failed to check if Sidewalk destination {name} exists')
        except ClientError as e:
            if e.response['Error']['Code'] == 'ResourceNotFoundException':
                return False
            else:
                terminate(f'Unable to get {name} destination: {e}.', ErrCode.EXCEPTION)

    def update_existing_destination(self, dest_name: str, role_name: str, interactive_mode: bool = True):
        """
        Updates existing destination for uplink messages from Sidewalk devices.

        :param dest_name:           Destination name.
        :param role_name:           Name of the role to be assigned to the destination.
        :param interactive_mode:    Turns the interactive mode on/off.
        """
        if interactive_mode:
            log_info(f'{dest_name} already exists and will be modified. Proceed?')
            confirm()
        else:
            log_info(f'{dest_name} already exists and will be modified.')
        try:
            log_info(f'Getting {role_name} role ARN...')
            response = self._iam_client.get_role(RoleName=f'{role_name}')
            log_success(f'{role_name} ARN obtained.')
            log_info(f'Updating {dest_name} destination...')
            role_arn = response['Role']['Arn']
            response = self._wireless_client.update_destination(
                Name=dest_name,
                ExpressionType='RuleName',
                Expression=f'{self._config.sid_demo_name}UplinkRule',
                Description='Destination for geolocation uplink messages from Sidewalk devices',
                RoleArn=role_arn
            )
            eval_client_response(response, f'{dest_name} destination updated.')
        except ClientError as e:
            terminate(f'Unable to update {dest_name} destination: {e}.', ErrCode.EXCEPTION)

    def reassign_role_to_destination(self, dest_name: str, role_name: str):
        """
        Checks if destination and role exist. If so, reassigns existing role to a given destination.

        Stacks can run simultaneously sharing the same destination;
        when one of the stacks is deleted (along with its destination role), the destination role of another one should
        be assigned to the destination, so that it keeps permission to publish to the sidewalk/app_data topic.

        :param dest_name:   Name of the destination.
        :param role_name:   Name of the role.
        """
        # Check if destination exists
        try:
            self._wireless_client.get_destination(Name=dest_name)
        except ClientError:
            return

        # Check if role exists and if so, reassign it to a destination
        try:
            response = self._iam_client.get_role(RoleName=f'{role_name}')
            role_arn = response['Role']['Arn']

            log_info(f'Reassigning {role_name} to the {dest_name} destination...')
            response = self._wireless_client.update_destination(
                Name=dest_name,
                ExpressionType='MqttTopic',
                Expression='sidewalk/app_data',
                Description='Destination for uplink messages from Sidewalk devices.',
                RoleArn=role_arn
            )
            eval_client_response(response, f'{dest_name} destination updated.')
        except (ClientError, KeyError):
            pass

    def check_if_destination_in_use(self, name: str) -> bool:
        """
        Checks if any Sidewalk device are using the given destination.

        :param name:    Destination name.
        :return:        True if exists, False otherwise.
        """
        try:
            log_info(f'Checking if {name} destination is used by any devices...')

            next_token = None
            associated_devices=[]

            while True:
                if next_token is None:
                    response = self._wireless_client.list_wireless_devices(DestinationName=name)
                else:
                    response = self._wireless_client.list_wireless_devices(NextToken=next_token, DestinationName=name)

                if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                    if 'WirelessDeviceList' in response:
                        for device in response['WirelessDeviceList']:
                            associated_devices.append(f'{device["Name"]} ({device["Id"]})')
                    if 'NextToken' in response:
                        next_token = str(response['NextToken'])
                    else:
                        next_token = None

                    if next_token is None:
                        break

            if len(associated_devices) > 0:
                log_warn(f'The following devices are still using destination {name}:')
                for device in associated_devices:
                    log_warn(f'    {device}')
                return True
            else:
                return False
        except Exception as e:
            terminate(f'Unable to check if {name} destination is in use: {e}.', ErrCode.EXCEPTION)

    def enable_notifications(self):
        """
        Enables notifications for Sidewalk devices
        """
        try:
            log_info('Enabling Sidewalk event notification in iotwireless...')
            response = self._wireless_client.update_event_configuration_by_resource_types(
                DeviceRegistrationState={'Sidewalk': {'WirelessDeviceEventTopic': 'Enabled'}},
                Proximity={'Sidewalk': {'WirelessDeviceEventTopic': 'Enabled'}},
                MessageDeliveryStatus={'Sidewalk': {'WirelessDeviceEventTopic': 'Enabled'}}
            )
            eval_client_response(response, 'Notifications enabled.')
        except ClientError as e:
            terminate(f'Notifications not enabled: {e}.', ErrCode.EXCEPTION)

    def get_sidewalk_device_profile(self, name: str):
        try:
            next_token = None

            while True:
                if next_token is None:
                    response = self._wireless_client.list_device_profiles(DeviceProfileType='Sidewalk')
                else:
                    response = self._wireless_client.list_device_profiles(NextToken=next_token, DeviceProfileType='Sidewalk')

                if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                    if 'DeviceProfileList' in response:
                        for profile_info in response['DeviceProfileList']:
                            if profile_info['Name'] == name:
                                # Stop search on first match
                                response = self._wireless_client.get_device_profile(Id=profile_info['Id'])
                                eval_client_response(response)
                                profile = {
                                    'Arn': response['Arn'],
                                    'Name': response['Name'],
                                    'Id': response['Id'],
                                    'Sidewalk': response['Sidewalk'],
                                }
                                return profile

                    if 'NextToken' in response:
                        next_token = str(response['NextToken'])
                    else:
                        next_token = None

                    if next_token is None:
                        break

            # No matching name was found
            return None
        except ClientError as e:
            terminate(f'Failed to find Sidewalk device profile: {e}.', ErrCode.EXCEPTION)

    def create_sidewalk_device_profile(self, name: str):
        try:
            log_info(f'Creating Sidewalk device profile {name}...')
            response = self._wireless_client.create_device_profile(Name=name, Sidewalk={})
            eval_client_response(response, f'Sidewalk device profile {name} created')
            self._config.set_wireless_device_profile_id(response['Id'])
        except ClientError as e:
            terminate(f'Failed to create Sidewalk device profile: {e}.', ErrCode.EXCEPTION)

    def delete_sidewalk_device_profile(self, name: str):
        try:
            log_info(f'Deleting Sidewalk device profile {name}...')

            # Try to get profile ID
            profile = self.get_sidewalk_device_profile(name)
            if profile is not None:
                response = self._wireless_client.delete_device_profile(Id=profile['Id'])
                eval_client_response(response, f'Sidewalk device profile {name} deleted')
            else:
                log_info(f'Sidewalk device profile {name} does not exist')
        except ClientError as e:
            terminate(f'Failed to delete Sidewalk device profile: {e}.', ErrCode.EXCEPTION)

    def check_if_sidewalk_device_profile_exists(self, name: str):
        try:
            log_info(f'Checking if Sidewalk device profile {name} exists...')

            # Try to get profile
            profile = self.get_sidewalk_device_profile(name)
            if profile is not None:
                return True
            else:
                return False
        except ClientError as e:
            terminate(f'Failed to delete Sidewalk device profile: {e}.', ErrCode.EXCEPTION)

    def check_if_device_profile_in_use(self, name: str) -> bool:
        """
        Checks if any Sidewalk device are using the given device profile.

        :param name:    Destination name.
        :return:        True if exists, False otherwise.
        """
        try:
            log_info(f'Checking if Sidewalk device profile {name} is used by any devices...')

            # Try to get profile ID
            profile = self.get_sidewalk_device_profile(name)
            if profile is None:
                log_warn(f'Sidewalk device profile {name} does not exist')
                return False

            next_token = None
            associated_devices=[]

            while True:
                if next_token is None:
                    response = self._wireless_client.list_wireless_devices(DeviceProfileId=profile['Id'])
                else:
                    response = self._wireless_client.list_wireless_devices(NextToken=next_token, DeviceProfileId=profile['Id'])

                if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                    if 'WirelessDeviceList' in response:
                        for device in response['WirelessDeviceList']:
                            associated_devices.append(f'{device["Name"]} ({device["Id"]})')
                    if 'NextToken' in response:
                        next_token = str(response['NextToken'])
                    else:
                        next_token = None

                    if next_token is None:
                        break
                else:
                    break

            if len(associated_devices) > 0:
                log_warn(f'The following devices are still using Sidewalk device profile {name}:')
                for device in associated_devices:
                    log_warn(f'    {device}')
                return True
            else:
                return False
        except Exception as e:
            terminate(f'Unable to check if Sidewalk device profile {name} is in use: {e}.', ErrCode.EXCEPTION)

    def get_sidewalk_device(self, device_name: str, profile_name: str):
        try:
            profile = self.get_sidewalk_device_profile(profile_name)
            if profile is None:
                raise AttributeError(f'Sidewalk device profile {profile_name} does not exist')

            next_token = None

            while True:
                if next_token is None:
                    response = self._wireless_client.list_wireless_devices(DeviceProfileId=profile['Id'])
                else:
                    response = self._wireless_client.list_wireless_devices(NextToken=next_token, DeviceProfileId=profile['Id'])

                if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                    if 'WirelessDeviceList' in response:
                        for device in response['WirelessDeviceList']:
                            if device['Name'] == device_name and device['Type'] == 'Sidewalk':
                                # Found a matching device
                                response = self._wireless_client.get_wireless_device(Identifier=device['Id'], IdentifierType='WirelessDeviceId')
                                eval_client_response(response, f'Found Sidewalk device {device_name} (WirelessDeviceId: {response["Id"]})')

                                device_data = {
                                    'Name': response['Name'],
                                    'Description': response['Description'],
                                    'DestinationName': response['DestinationName'],
                                    'Id': response['Id'],
                                    'Arn': response['Arn'],
                                    'Sidewalk': response['Sidewalk']
                                }
                                return device_data

                    if 'NextToken' in response:
                        next_token = str(response['NextToken'])
                    else:
                        next_token = None

                    if next_token is None:
                        break
                else:
                    break

            # No matching device was found
            return None
        except Exception as e:
            terminate(f'Failed to find Sidewalk device profile: {e}.', ErrCode.EXCEPTION)

    def check_if_sidewalk_device_exists(self, device_name: str, device_profile: str) -> bool:
        device = self.get_sidewalk_device(device_name, device_profile)
        return device is not None

    def create_sidewalk_device(self, device_name: str, profile_name: str, destination_name: str):
        try:
            log_info(f'Creating Sidewalk device named {device_name}...')

            profile = self.get_sidewalk_device_profile(profile_name)
            if profile is None:
                raise AttributeError(f'Sidewalk device profile {profile_name} does not exist')

            if not self.check_if_destination_exists(destination_name):
                raise AttributeError(f'Sidewalk destination {destination_name} does not exist')

            response = self._wireless_client.create_wireless_device(
                Type='Sidewalk',
                Name=device_name,
                Description=self._constants.SIDEWALK_DEVICE_DESCRIPTION,
                DestinationName=destination_name,
                Sidewalk={
                    'DeviceProfileId': profile['Id']
                },
                Tags=[
                    {
                        'Key': 'string',
                        'Value': self._constants.SIDEWALK_DEMO_TAG
                    }
                ]
            )
            eval_client_response(response, f'Sidewalk device {device_name} successfully created')

            # Get full device data
            response = self._wireless_client.get_wireless_device(Identifier=response['Id'], IdentifierType='WirelessDeviceId')
            if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                device_data = {
                    'Name': response['Name'],
                    'Description': response['Description'],
                    'DestinationName': response['DestinationName'],
                    'Id': response['Id'],
                    'Arn': response['Arn'],
                    'Sidewalk': response['Sidewalk']
                }
                return device_data
            else:
                raise IOError(f'Failed to get device info from the cloud')
        except Exception as e:
            terminate(f'Failed to create Sidewalk device: {e}.', ErrCode.EXCEPTION)

    def delete_sidewalk_device(self, device_name: str, device_id: str):
        try:
            log_info(f'Deleting Sidewalk device {device_name}...')

            # Ensure the device is deregistered
            self._wireless_client.deregister_wireless_device(Identifier=device_id, WirelessDeviceType='Sidewalk')

            # Delete the device
            response = self._wireless_client.delete_wireless_device(Id=device_id)
            eval_client_response(response, f'Sidewalk device {device_name} deleted')
        except ClientError as e:
            terminate(f'Failed to delete Sidewalk device: {e}.', ErrCode.EXCEPTION)

    def set_sidewalk_device_destination(self, device_name: str, device_id: str, destination_name: str):
        try:
            log_info(f'Changing Sidewalk device {device_name} destination to {destination_name}...')

            if not self.check_if_destination_exists(destination_name):
                raise AttributeError(f'Sidewalk destination {destination_name} does not exist')

            response = self._wireless_client.update_wireless_device(
                Id=device_id,
                DestinationName=destination_name,
                Description=self._constants.SIDEWALK_DEVICE_DESCRIPTION,
            )
            eval_client_response(response, f'Successfully changed Sidewalk device {device_name} destination to {destination_name}')
        except Exception as e:
            terminate(f'Failed to set Sidewalk device {device_name} destination to {device_name}: {e}.', ErrCode.EXCEPTION)
