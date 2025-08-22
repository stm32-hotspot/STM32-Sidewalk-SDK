#
##############################################################################
# file:    boto_session.py
# brief:   Helper class to instantiate Boto3 session using the supplied creds
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


from boto3 import Session
from libs.config import Config
from libs.utils import *


class BotoSession(Session):
    """
    A singleton class to keep AWS session running using the specified credentials
    """
    __instance = None
    __init_done = False

    def __new__(cls, config: Config=None):
        if cls.__instance is None:
            cls.__instance = super(BotoSession, cls).__new__(cls)
        return cls.__instance

    def __init__(self, config: Config=None):
        if not BotoSession.__init_done:
            if config is None:
                raise AttributeError('Configuration cannot be empty when BotoSession is instantiated for the very first time')

            log_info('Trying to contact AWS...')
            super().__init__(profile_name=config.aws_profile,
                             aws_access_key_id=config.aws_access_key_id,
                             aws_secret_access_key=config.aws_secret_access_key,
                             aws_session_token=config.aws_session_token,
                             region_name=config.aws_default_region,
            )
            log_success('Established connection with AWS using the supplied credentials')
            BotoSession.__init_done = True
