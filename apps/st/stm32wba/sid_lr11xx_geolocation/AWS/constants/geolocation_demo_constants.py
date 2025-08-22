#
##############################################################################
# file:    geolocation_demo_constants.py
# brief:   Helper class to host constants for demo deployment
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

from helpers.boto_session import BotoSession
from libs.config import Config
from os import path


class GeolocationDemoConstants:
    """
    A singleton class that hosts global constants for Geolocation Demo app
    """
    __instance = None
    __init_done = False

    def __new__(cls, config: Config=None):
        if cls.__instance is None:
            cls.__instance = super(GeolocationDemoConstants, cls).__new__(cls)
        return cls.__instance

    def __init__(self, config: Config=None):
        if not GeolocationDemoConstants.__init_done:
            sts = BotoSession(config).client('sts')
            caller_identity = sts.get_caller_identity()

            # Generic
            self.AWS_ACCOUNT_ID = caller_identity["Account"]
            self.SIDEWALK_DEMO_STACK_NAME = f'{config.sid_demo_name}Stack'
            self.SIDEWALK_DEMO_TAG = f'{config.sid_demo_name}'
            self.CLOUD_TEMPLATE_ROOT_DIR = str(path.join(path.dirname(__file__), '..', 'CloudDeployment'))

            # CloudFormation
            self.CLOUDFORMATION_TEMPLATE_SRC_ROOT_DIR = str(path.join(self.CLOUD_TEMPLATE_ROOT_DIR, 'cloudformation'))
            self.CLOUDFORMATION_TEMPLATE_SRC_FILENAME = 'SidewalkGeolocationStack.yaml'
            self.CLOUDFORMATION_TEMPLATE_DST_FILENAME = f'{self.SIDEWALK_DEMO_STACK_NAME}.yaml'
            self.CLOUDFORMATION_TEMPLATE_BUCKET_NAME = f'{self.AWS_ACCOUNT_ID}-sidewalk-cloudformation-template'
            self.CLOUDFORMATION_TEMPLATE_BUCKET_URL = f'https://{self.CLOUDFORMATION_TEMPLATE_BUCKET_NAME}.s3.amazonaws.com/{self.CLOUDFORMATION_TEMPLATE_DST_FILENAME}'

            # IoT Core for Sidewalk
            self.SIDEWALK_DESTINATION_NAME = f'{config.sid_demo_name}Destination'
            self.SIDEWALK_DESTINATION_ROLE = f'{config.sid_demo_name}DestinationRole'
            self.SIDEWALK_DEVICE_PROFILE = f'{config.sid_demo_name}DeviceProfile'
            self.SIDEWALK_DEVICE_DESCRIPTION = f'{config.sid_demo_name} demo device that sends its geolocation scan data over Sidewalk'

            # Lambda-related
            self.LAMBDA_SRC_ROOT_DIR = str(path.join(self.CLOUD_TEMPLATE_ROOT_DIR, 'lambda'))

            GeolocationDemoConstants.__init_done = True
