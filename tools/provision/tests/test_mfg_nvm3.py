#!/usr/bin/env python3
#
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
#

import unittest
import json
import yaml
from sid_provision.run import SidMfgAwsJson
from sid_provision.run import SidMfgAcsJson
from sid_provision.run import SidMfgOutNVM3
from sid_provision.run import AttrDict
from unittest.mock import patch, mock_open

import os

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


class TestSidMfgNVM3(unittest.TestCase):
    def setUp(self):
        """
        Setup the helper files that are used for unit testing, NVM3 file
        generation from acs console json files
        """

        self._test_data_acs = None
        self._test_config_acs = None
        self._encoded_acs_console = None
        self._test_data_aws = None
        self._test_config_aws = None
        self._encoded_aws_console = None
        self._test_file_name = "mfg.nvm3"

        with open(os.path.join(THIS_DIR, "test_data_acs.json"), "r") as _:
            self._test_data_acs = AttrDict(json.load(_))

        with open(os.path.join(THIS_DIR, "test_config_acs.yaml"), "r") as _:
            self._test_config_acs = AttrDict(yaml.safe_load(_))

        with open(os.path.join(THIS_DIR, "test_mfg_acs_json.nvm3"), "r") as _:
            self._encoded_acs_console = _.read().rstrip()

        with open(os.path.join(THIS_DIR, "test_data_aws.json"), "r") as _:
            self._test_data_aws = AttrDict(json.load(_))

        with open(os.path.join(THIS_DIR, "test_config_aws.yaml"), "r") as _:
            self._test_config_aws = AttrDict(yaml.safe_load(_))

        with open(os.path.join(THIS_DIR, "test_mfg_aws_json.nvm3"), "r") as _:
            self._encoded_aws_console = _.read().rstrip()

    def test_acs_json_sanity(self):
        """
        Sanity test mfg data created from a test acs json file
        """
        mfg_data = SidMfgAcsJson(
            acs_json=self._test_data_acs.acs.json,
            app_pub=self._test_data_acs.app_pub,
            config=self._test_config_acs,
        )

        with patch("builtins.open", mock_open(read_data=b"")) as mocked_file:
            with SidMfgOutNVM3(self._test_file_name) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "w+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console)

    def test_aws_json_sanity(self):
        """
        Sanity test mfg data created from test aws json files
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json=self._test_data_aws.aws.wireless_device_json,
            aws_device_profile_json=self._test_data_aws.aws.device_profile_json,
            aws_certificate_json={},
            config=self._test_config_aws,
        )

        with patch("builtins.open", mock_open(read_data=b"")) as mocked_file:
            with SidMfgOutNVM3(self._test_file_name) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "w+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console)

    def test_mfg_data_cannot_be_none(self):
        """
        Sanity test mfg data must be not be none
        """

        with patch("builtins.open", mock_open(read_data=b"")) as mocked_file:
            with SidMfgOutNVM3(self._test_file_name) as out_file:
                self.assertRaises(Exception, out_file.write, None)
