#!/usr/bin/env python3
#
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
#

import unittest
import binascii
from sid_provision.run import SidMfgObj
from sid_provision.run import AttrDict


class TestSidMfgObj(unittest.TestCase):
    def test_positive_cases(self):
        """
        Test the various types of input values that can be given to SidMfgObj,
        and check if SidMfgObj can convert the value to a bytes as exposed by
        the encoded field.
        """

        word_size = 8
        info = AttrDict({"start": 20, "end": 21})
        mfg_enum = AttrDict({"name": "Int Value", "value": 4, "size": 8})
        obj = SidMfgObj(mfg_enum=mfg_enum, value=45, info=info, word_size=word_size)
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(binascii.hexlify(obj.encoded), b"000000000000002d")

        word_size = 4
        info = AttrDict({"start": 0, "end": 1})
        mfg_enum = AttrDict({"name": "Int Value", "value": 4, "size": 4})
        obj = SidMfgObj(mfg_enum=mfg_enum, value=7, info=info, word_size=word_size)
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(binascii.hexlify(obj.encoded), b"00000007")

        word_size = 4
        info = AttrDict({"start": 0, "end": 1})
        mfg_enum = AttrDict({"name": "Int Value", "value": 5, "size": 4})
        obj = SidMfgObj(
            mfg_enum=mfg_enum,
            value=7,
            info=info,
            word_size=word_size,
            is_network_order=False,
        )
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(binascii.hexlify(obj.encoded), b"07000000")

        value_bytearray = bytearray([0xDE, 0xED, 0xBE, 0xEF])
        word_size = 2
        info = AttrDict({"start": 40, "end": 45})
        mfg_enum = AttrDict({"name": "Byte Array", "value": 4, "size": 10})
        obj = SidMfgObj(
            mfg_enum=mfg_enum,
            value=value_bytearray,
            info=info,
            word_size=word_size,
        )
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(binascii.hexlify(obj.encoded), b"deedbeef000000000000")

        word_size = 2
        info = AttrDict({"start": 40, "end": 45})
        mfg_enum = AttrDict({"name": "Bytes", "value": 5, "size": 10})
        obj = SidMfgObj(
            mfg_enum=mfg_enum,
            value=bytes(value_bytearray),
            info=info,
            word_size=word_size,
        )
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(binascii.hexlify(obj.encoded), b"deedbeef000000000000")

        class ByteLike:
            def __init__(self, value):
                self._value = value

            def __bytes__(self):
                return bytes(self._value)

        value_byte_like = ByteLike(
            bytearray(
                [
                    0x00,
                    0x11,
                    0x22,
                    0x33,
                    0x44,
                    0x55,
                    0x66,
                    0x77,
                    0x88,
                    0x99,
                    0xAA,
                    0xBB,
                    0xCC,
                    0xDD,
                    0xEE,
                    0xFF,
                    0xFF,
                    0xEE,
                    0xDD,
                    0xCC,
                    0xBB,
                    0xAA,
                    0x99,
                    0x88,
                    0x77,
                    0x66,
                    0x55,
                    0x44,
                    0x33,
                    0x22,
                    0x11,
                    0x00,
                ]
            )
        )
        word_size = 4
        info = AttrDict({"start": 100, "end": 108})
        mfg_enum = AttrDict({"name": "Bytes Like", "value": 4, "size": 32})
        obj = SidMfgObj(
            mfg_enum=mfg_enum,
            value=value_byte_like,
            info=info,
            word_size=word_size,
        )
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(
            binascii.hexlify(obj.encoded),
            b"00112233445566778899aabbccddeeffffeeddccbbaa99887766554433221100",
        )

        string_value = "Mr. Owl ate my metal worm"
        word_size = 16
        info = AttrDict({"start": 900, "end": 902})
        mfg_enum = AttrDict({"name": "String", "value": 4, "size": 32})
        obj = SidMfgObj(mfg_enum, value=string_value, info=info, word_size=word_size)
        self.assertEqual(obj.start, info.start * word_size)
        self.assertEqual(obj.end, info.end * word_size)
        self.assertEqual(
            binascii.hexlify(obj.encoded),
            b"4d722e204f776c20617465206d79206d6574616c20776f726d00000000000000",
        )

    def test_negative_cases(self):
        """
        Test the handling of invalid input given to SidMfgObj, and test for its
        behavior.
        """

        generic_info = {"start": 0, "end": 1}
        mfg_enum = AttrDict({"name": "Generic", "value": 4, "size": 8})

        with self.assertRaises(AssertionError):
            SidMfgObj(mfg_enum, value=4, info={}, word_size=10)

        with self.assertRaises(AttributeError):
            SidMfgObj(mfg_enum, value=4, info={"foo": "bar"}, word_size=10)

        invalid_info_offset_equal = {"start": 0, "end": 0}
        with self.assertRaises(AssertionError):
            SidMfgObj(mfg_enum, value=4, info=invalid_info_offset_equal, word_size=10)
        invalid_info = {"start": 1, "end": 0}
        with self.assertRaises(AssertionError):
            SidMfgObj(mfg_enum, value=4, info=invalid_info, word_size=10)

        with self.assertRaises(AssertionError):
            SidMfgObj(mfg_enum, value=4, info=generic_info, word_size="10")

        class Invalid:
            def __init__(self, value):
                self._value = value

        with self.assertRaises(ValueError):
            SidMfgObj(mfg_enum, value=Invalid(4), info=generic_info, word_size=10)

        with self.assertRaises(ValueError):
            SidMfgObj(mfg_enum, bytearray([0]) * 32, generic_info, 10)


if __name__ == "__main__":
    unittest.main()
