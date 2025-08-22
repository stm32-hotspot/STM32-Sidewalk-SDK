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
from sid_provision.run import SidMfgBBJson
from sid_provision.run import SidMfgOutBin
from sid_provision.run import AttrDict
from unittest.mock import patch, mock_open

import os

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


class TestSidMfgBin(unittest.TestCase):
    def setUp(self):
        """
        Setup the helper files that are used for unit testing, bin file
        generation from black box, acs console json files and aws json files
        """

        self._test_data_acs = None
        self._test_data_aws = None
        self._test_data_bb = None
        self._test_data_acs_serial_expansion = None
        self._test_config_acs = None
        self._test_config_aws = None
        self._test_config_bb = None
        self._test_config_acs_serial_expansion = None
        self._encoded_acs_console = None
        self._encoded_aws_console = None
        self._encoded_bb = None
        self._encoded_acs_console_serial_expansion = None
        self._test_file_name = "test_file.bin"

        with open(os.path.join(THIS_DIR, "test_data_acs.json"), "r") as _:
            self._test_data_acs = AttrDict(json.load(_))

        with open(os.path.join(THIS_DIR, "test_data_aws.json"), "r") as _:
            self._test_data_aws = AttrDict(json.load(_))

        with open(os.path.join(THIS_DIR, "test_data_bb.json"), "r") as _:
            self._test_data_bb = AttrDict(json.load(_))

        with open(os.path.join(THIS_DIR, "test_data_acs_serial_expansion.json"), "r") as _:
            self._test_data_acs_serial_expansion = AttrDict(json.load(_))

        with open(os.path.join(THIS_DIR, "test_config_acs.yaml"), "r") as _:
            self._test_config_acs = AttrDict(yaml.safe_load(_))

        with open(os.path.join(THIS_DIR, "test_config_aws.yaml"), "r") as _:
            self._test_config_aws = AttrDict(yaml.safe_load(_))

        with open(os.path.join(THIS_DIR, "test_config_bb.yaml"), "r") as _:
            self._test_config_bb = AttrDict(yaml.safe_load(_))

        with open(os.path.join(THIS_DIR, "test_config_acs_serial_expansion.yaml"), "r") as _:
            self._test_config_acs_serial_expansion = AttrDict(yaml.safe_load(_))

        with open(os.path.join(THIS_DIR, "test_prefilled_data_large.bin"), "rb") as _:
            self._test_prefilled_data_large = _.read()

        with open(os.path.join(THIS_DIR, "test_prefilled_data_small.bin"), "rb") as _:
            self._test_prefilled_data_small = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_acs_json.bin"), "rb") as _:
            self._encoded_acs_console = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_acs_json_prefilled_small.bin"), "rb") as _:
            self._encoded_acs_console_prefilled_small = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_acs_json_prefilled_large.bin"), "rb") as _:
            self._encoded_acs_console_prefilled_large = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_acs_json_serial_expansion.bin"), "rb") as _:
            self._encoded_acs_console_serial_expansion = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_acs_json_serial_expansion_prefilled.bin"), "rb") as _:
            self._encoded_acs_console_serial_expansion_prefilled = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_aws_json.bin"), "rb") as _:
            self._encoded_aws_console = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_aws_json_prefilled_small.bin"), "rb") as _:
            self._encoded_aws_console_prefilled_small = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_aws_json_prefilled_large.bin"), "rb") as _:
            self._encoded_aws_console_prefilled_large = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_bb_json.bin"), "rb") as _:
            self._encoded_bb = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_bb_json_prefilled_small.bin"), "rb") as _:
            self._encoded_bb_prefilled_small = _.read()

        with open(os.path.join(THIS_DIR, "test_mfg_bb_json_prefilled_large.bin"), "rb") as _:
            self._encoded_bb_prefilled_large = _.read()

    def test_aws_json_sanity(self):
        """
        Sanity test mfg data created from a test aws json files
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json=self._test_data_aws.aws.wireless_device_json,
            aws_device_profile_json=self._test_data_aws.aws.device_profile_json,
            aws_certificate_json={},
            config=self._test_config_aws,
        )

        with patch("builtins.open", mock_open(read_data=b"")) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console)

    def test_aws_json_sanity_prefill_large(self):
        """
        Sanity test mfg data created from a test aws json file with prefilled
        data (larger than the expected encoded size)
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json=self._test_data_aws.aws.wireless_device_json,
            aws_device_profile_json=self._test_data_aws.aws.device_profile_json,
            aws_certificate_json={},
            config=self._test_config_aws,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_aws_console_prefilled_large),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console_prefilled_large)

    def test_aws_json_sanity_prefill_small(self):
        """
        Sanity test mfg data created from a test aws json files with prefilled
        data (smaller than the expected encoded size)
        """

        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json=self._test_data_aws.aws.wireless_device_json,
            aws_device_profile_json=self._test_data_aws.aws.device_profile_json,
            aws_certificate_json={},
            config=self._test_config_aws,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_aws_console_prefilled_small),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console_prefilled_small)

    def test_aws_json_sanity_prefill_equal(self):
        """
        Sanity test mfg data created from a test aws json file with prefilled
        data (same as the encoded size). For simplicity sake using the prefined
        aws json bin
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json=self._test_data_aws.aws.wireless_device_json,
            aws_device_profile_json=self._test_data_aws.aws.device_profile_json,
            aws_certificate_json={},
            config=self._test_config_aws,
        )

        with patch("builtins.open", mock_open(read_data=self._encoded_aws_console)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console)

    def test_aws_certificate_json_sanity(self):
        """
        Sanity test mfg data created from test aws certificate json files
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json={},
            aws_device_profile_json={},
            aws_certificate_json=self._test_data_aws.aws.certificate_json,
            config=self._test_config_aws,
        )

        with patch("builtins.open", mock_open(read_data=b"")) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console)

    def test_aws_certificate_json_sanity_prefill_large(self):
        """
        Sanity test mfg data created from test certificate aws json file with prefilled
        data (larger than the expected encoded size)
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json={},
            aws_device_profile_json={},
            aws_certificate_json=self._test_data_aws.aws.certificate_json,
            config=self._test_config_aws,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_aws_console_prefilled_large),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console_prefilled_large)

    def test_aws_certificate_json_sanity_prefill_small(self):
        """
        Sanity test mfg data created from test certificate aws json files with prefilled
        data (smaller than the expected encoded size)
        """

        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json={},
            aws_device_profile_json={},
            aws_certificate_json=self._test_data_aws.aws.certificate_json,
            config=self._test_config_aws,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_aws_console_prefilled_small),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console_prefilled_small)

    def test_aws_certificate_json_sanity_prefill_equal(self):
        """
        Sanity test mfg data created from test aws certificate json file with prefilled
        data (same as the encoded size). For simplicity sake using the prefined
        aws json bin
        """
        mfg_data = SidMfgAwsJson(
            aws_wireless_device_json={},
            aws_device_profile_json={},
            aws_certificate_json=self._test_data_aws.aws.certificate_json,
            config=self._test_config_aws,
        )

        with patch("builtins.open", mock_open(read_data=self._encoded_aws_console)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_aws) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_aws_console)

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
            with SidMfgOutBin(self._test_file_name, self._test_config_acs) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console)

    def test_acs_json_sanity_prefill_large(self):
        """
        Sanity test mfg data created from a test acs json file with prefilled
        data (larger than the expected encoded size)
        """
        mfg_data = SidMfgAcsJson(
            acs_json=self._test_data_acs.acs.json,
            app_pub=self._test_data_acs.app_pub,
            config=self._test_config_acs,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_acs_console_prefilled_large),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_acs) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console_prefilled_large)

    def test_acs_json_sanity_prefill_small(self):
        """
        Sanity test mfg data created from a test acs json file with prefilled
        data (smaller than the expected encoded size)
        """
        mfg_data = SidMfgAcsJson(
            acs_json=self._test_data_acs.acs.json,
            app_pub=self._test_data_acs.app_pub,
            config=self._test_config_acs,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_acs_console_prefilled_small),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_acs) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console_prefilled_small)

    def test_acs_json_sanity_prefill_equal(self):
        """
        Sanity test mfg data created from a test acs json file with prefilled
        data (same as the encoded size). For simplicity sake using the prefined
        acs json bin
        """
        mfg_data = SidMfgAcsJson(
            acs_json=self._test_data_acs.acs.json,
            app_pub=self._test_data_acs.app_pub,
            config=self._test_config_acs,
        )

        with patch("builtins.open", mock_open(read_data=self._encoded_acs_console)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_acs) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console)

    def test_bb_json_sanity(self):
        """
        Sanity test mfg data created from a test BB json file
        """
        mfg_data = SidMfgBBJson(self._test_data_bb, self._test_config_bb)

        with patch("builtins.open", mock_open(read_data=self._encoded_bb)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_bb) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_bb)

    def test_bb_json_sanity_prefill_large(self):
        """
        Sanity test mfg data created from a test bb json file with prefilled
        data (larger than the expected encoded size)
        """
        mfg_data = SidMfgBBJson(
            bb_json=self._test_data_bb,
            config=self._test_config_bb,
        )

        with patch("builtins.open", mock_open(read_data=self._encoded_bb_prefilled_large)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_bb) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_bb_prefilled_large)

    def test_bb_json_sanity_prefill_small(self):
        """
        Sanity test mfg data created from a test bb json file with prefilled
        data (smaller than the expected encoded size)
        """
        mfg_data = SidMfgBBJson(
            bb_json=self._test_data_bb,
            config=self._test_config_bb,
        )

        with patch("builtins.open", mock_open(read_data=self._encoded_bb_prefilled_small)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_bb) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_bb_prefilled_small)

    def test_bb_json_sanity_prefill_equal(self):
        """
        Sanity test mfg data created from a test bb json file with prefilled
        data (same as the encoded size). For simplicity sake using the prefined
        acs json bin
        """
        mfg_data = SidMfgBBJson(
            bb_json=self._test_data_bb,
            config=self._test_config_bb,
        )

        with patch("builtins.open", mock_open(read_data=self._encoded_bb_prefilled_small)) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_bb) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_bb_prefilled_small)

    def test_acs_json_serial_expansion_sanity(self):
        """
        Sanity test mfg data created from a test acs json file with long serial numbers, and therefore using TLV MFG
        """
        mfg_data = SidMfgAcsJson(
            acs_json=self._test_data_acs_serial_expansion.acs.json,
            app_pub=self._test_data_acs_serial_expansion.app_pub,
            config=self._test_config_acs_serial_expansion,
        )

        with patch("builtins.open", mock_open(read_data=b"")) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_acs_serial_expansion) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console_serial_expansion)

    def test_acs_json_serial_expansion_sanity_prefill(self):
        """
        Sanity test mfg data created from a test acs json file with prefilled and long serial numbers, and therefore using TLV MFG
        """
        mfg_data = SidMfgAcsJson(
            acs_json=self._test_data_acs_serial_expansion.acs.json,
            app_pub=self._test_data_acs_serial_expansion.app_pub,
            config=self._test_config_acs_serial_expansion,
        )

        with patch(
            "builtins.open",
            mock_open(read_data=self._encoded_acs_console_serial_expansion_prefilled),
        ) as mocked_file:
            with SidMfgOutBin(self._test_file_name, self._test_config_acs_serial_expansion) as out_file:
                out_file.write(mfg_data)

            mocked_file.assert_called_once_with(self._test_file_name, "wb+")
            mocked_file().write.assert_called_once_with(self._encoded_acs_console_serial_expansion)
