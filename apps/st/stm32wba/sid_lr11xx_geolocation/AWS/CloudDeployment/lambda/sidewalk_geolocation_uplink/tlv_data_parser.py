#
##############################################################################
# file:    tlv_data_parser.py
# brief:   Parser of Tag-Length-Value (TLV) payload in geolocation uplinks
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

from datetime import datetime, timezone


class TlvParser:
    """
    Decodes Tag-Length-Value (TLV) payload from the end node and transforms it into a usable Python object
    """

    __TAG_COUNTER_VALUE        = 'c7'  # Demo Counter - just a sample of numerical value, has no relation to geolocation functionality
    __TAG_WIFI_SCAN_DATA       = '3f'  # WiFi scan data from LR11xx - includes BSSIDs of nearby WiFi networks with associated RSSI for each of them
    __TAG_GNSS_NO_AP_SCAN_DATA = '62'  # GNSS scan data from LR11xx when a cold start (no Assist Position) scan method was used
    __TAG_GNSS_AP_SCAN_DATA    = '63'  # GNSS scan data from LR11xx when Assist Position scan mode was used - includes Assist Position used by LR11xx during the scan

    @staticmethod
    def parse(payload: str):
        # Ensure payload hex string is lowercase
        payload = payload.lower()

        # Create empty dictionary for parsed data
        geolocation_data = {}

        while payload:
            tag            = payload[:2]
            length         = int(payload[2:4], 16)
            hex_str_length = length*2
            raw_data       = payload[4:4+hex_str_length]
            payload        = payload[4+hex_str_length:]

            if tag == TlvParser.__TAG_WIFI_SCAN_DATA:
                print(f'Parsing {length} bytes of WiFi scan data: {raw_data}')
                geolocation_data['wifi_scans'] = TlvParser.__parse_wifi_scan_data(raw_data)
            elif tag == TlvParser.__TAG_GNSS_NO_AP_SCAN_DATA:
                print(f'Parsing {length} bytes of GNSS (no assist position) scan data: {raw_data}')
                geolocation_data['gnss_scans'] = TlvParser.__parse_gnss_scan_data(raw_data)
            elif tag == TlvParser.__TAG_GNSS_AP_SCAN_DATA:
                print(f'Parsing {length} bytes of GNSS (with assist position) scan data: {raw_data}')
                geolocation_data['gnss_scans'] = TlvParser.__parse_gnss_with_ap_scan_data(raw_data)
            elif tag == TlvParser.__TAG_COUNTER_VALUE:
                # Don't do anything specific about the demo counter, print it to the logs and that's it
                print(f'Received demo counter value: {int(raw_data, 16)}')
            else:
                print(f'Unknown TLV tag: {tag} - skipped')

        return geolocation_data

    @staticmethod
    def __parse_wifi_scan_data(raw_data: str):
        wifi_scans=[]

        while raw_data:
            mac_address = raw_data[:12]
            raw_data    = raw_data[12:]
            # Format MAC address - insert octet delimiters to MAC address and capitalize octets
            mac_address = ':'.join(mac_address[i:i+2].upper() for i in range(0, len(mac_address), 2))
            print(f'Parsed MAC address: {mac_address}')

            rssi        = int.from_bytes(bytes.fromhex(raw_data[:2]), signed=True)
            raw_data    = raw_data[2:]
            print(f'Parsed RSSI: {rssi}')

            wifi_scans.append({'mac_address': mac_address, 'rssi': rssi})

        return wifi_scans

    @staticmethod
    def __convert_raw_timestamp_to_gps_timestamp(raw_timestamp: int):
        # Convert raw timestamp to GPS timestamp
        # GPS timestamp is the number of seconds since January 6, 1980
        # Raw timestamp is the number of seconds since January 6, 1980 with modulo of 1024 weeks

        # Restore missing seconds due to 1024 week modulo

        # Get current UTC time
        current_utc_time = datetime.now(timezone.utc)

        # UTC January 6, 1980
        january_6_1980 = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

        # Calculate the difference between current UTC time and January 6, 1980
        time_difference = current_utc_time - january_6_1980

        # Seconds per week
        seconds_per_week = 60 * 60 * 24 * 7

        # Calculate how many times 1024 weeks fit
        full_1024_weeks = time_difference.total_seconds() // seconds_per_week // 1024

        # Compute the GPS timestamp
        gps_timestamp = int(raw_timestamp + full_1024_weeks * seconds_per_week * 1024)

        return gps_timestamp

    @staticmethod
    def __parse_gnss_scan_data(raw_data: str):
        gnss_scans=[]

        # Bytes 0-1: GNSS scan timestamp accuracy (in ms)
        gnss_scan_time_accuracy = float(int(raw_data[:4], 16) + 1) / 4096.0  # Convert back from 0.244ms/LSB to fractional ms
        raw_data                = raw_data[4:]
        print(f'Parsed GNSS time accuracy: {gnss_scan_time_accuracy}')

        while raw_data:
            # Bytes 0-3: GNSS scan timestamp (Big Endian)
            gnss_scan_raw_timestamp = int(raw_data[:8], 16)
            raw_data                = raw_data[8:]
            print(f'Parsed GNSS scan raw timestamp: {gnss_scan_raw_timestamp}')
            gnss_scan_timestamp = TlvParser.__convert_raw_timestamp_to_gps_timestamp(gnss_scan_raw_timestamp)
            print(f'Recalculated GNSS scan timestamp: {gnss_scan_timestamp}')

            # Byte 4: length of the NAV3 data in this record
            nav3_data_length = int(raw_data[:2], 16)
            raw_data         = raw_data[2:]
            print(f'Parsed NAV3 data length: {nav3_data_length}')

            # Bytes 5..n: NAV3 data
            nav3_data = raw_data[:nav3_data_length*2]
            raw_data  = raw_data[nav3_data_length*2:]
            print(f'Parsed NAV3 data: {nav3_data}')

            gnss_scans.append({
                'timestamp':     gnss_scan_timestamp,
                'time_accuracy': gnss_scan_time_accuracy,
                'nav3_payload':  nav3_data
            })

        return gnss_scans

    @staticmethod
    def __parse_gnss_with_ap_scan_data(raw_data: str):
        # Bytes 0-1: GNSS position latitude (with 0.044° resolution)
        ap_latitude = float(int.from_bytes(bytes.fromhex(raw_data[:4]), signed=True)) * 90.0 / 2048.0
        raw_data    = raw_data[4:]

        # Bytes 2-3: GNSS position longitude (with 0.088° resolution)
        ap_longitude = float(int.from_bytes(bytes.fromhex(raw_data[:4]), signed=True)) * 180.0 / 2048.0
        raw_data     = raw_data[4:]
        print(f'Parsed GNSS assist position: [{ap_latitude}, {ap_longitude}]')

        # The rest of the payload is identical to the message with no assist position
        gnss_scans = TlvParser.__parse_gnss_scan_data(raw_data)

        # Append assist position to every GNSS scan
        for scan in gnss_scans:
            scan['assist_position'] = [
                ap_latitude,
                ap_longitude
            ]

        return gnss_scans
