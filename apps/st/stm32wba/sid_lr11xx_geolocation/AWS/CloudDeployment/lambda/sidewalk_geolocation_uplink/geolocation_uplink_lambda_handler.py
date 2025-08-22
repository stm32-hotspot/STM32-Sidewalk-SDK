#
##############################################################################
# file:    geolocation_uplink_lambda_handler.py
# brief:   Geolocation Sidewalk uplink processor and location resolver
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

import json
import base64
import boto3
import os
import traceback

from fragments_assembler import *
from tlv_data_parser import *


__SEMTECH_GNSS_RESOLVER_CUTOFF_TIME_ACCURACY: int = 16000  # GNSS resolver accepts assist time with accuracy better than 16 seconds, otherwise provided assist time is ignored


def try_resolve_wifi_position(wifi_scan_payload):
    resolved_position = None

    try:
        iotwireless_client = boto3.client('iotwireless')

        # Resolve position using WiFi scans
        get_position_estimate_response = iotwireless_client.get_position_estimate(
            WiFiAccessPoints=wifi_scan_payload
        )

        # Check the request was processed successfully
        if get_position_estimate_response['ResponseMetadata']['HTTPStatusCode'] == 200:
            # Read response body and convert into JSON object
            wifi_based_position = json.loads(get_position_estimate_response['GeoJsonPayload'].read().decode("utf-8"))

            # Report back the resolved position
            resolved_position = wifi_based_position
        else:
            print(f'Unable to resolve WiFi position: {get_position_estimate_response}')

    # React on ResourceNotFoundException
    except iotwireless_client.exceptions.ResourceNotFoundException as e:
        # Typically this means the position cannot be calculated reliably from the supplied scan data (e.g. insufficient WiFi BSSIDs supplied)
        print(f'Unable to resolve WiFi position: {e}')

    except Exception:
        print(f'Unexpected error occurred when resolving WiFi scans: {traceback.format_exc()}')

    return resolved_position


def try_resolve_gnss_position(gnss_scan_payload):
    resolved_position = None

    try:
        iotwireless_client = boto3.client('iotwireless')

        get_position_estimate_response=iotwireless_client.get_position_estimate(
            Gnss=gnss_scan_payload
        )

        # Check the request was processed successfully
        if get_position_estimate_response['ResponseMetadata']['HTTPStatusCode'] == 200:
            # Read response body and convert into JSON object
            gnss_based_position = json.loads(get_position_estimate_response['GeoJsonPayload'].read().decode("utf-8"))

            # Report back the resolved position
            resolved_position = gnss_based_position
        else:
            print(f'Unable to resolve GNSS position: {get_position_estimate_response}')

    # React on ResourceNotFoundException
    except iotwireless_client.exceptions.ResourceNotFoundException as e:
        # Typically this means the position cannot be calculated reliably from the supplied scan data (e.g. insufficient satellites detected)
        print(f'Unable to resolve GNSS position: {e}')

    except Exception:
        print(f'Unexpected error occurred when resolving GNSS scan: {traceback.format_exc()}')

    return resolved_position


def lambda_handler(event, context):
    try:
        # ---------------------------------------------
        # Read the environment variables
        # ---------------------------------------------
        geolocation_mqtt_topic_base = os.environ.get('GEOLOCATION_MQTT_TOPIC_BASE')

        # ---------------------------------------------------------------
        # Receive and record incoming event in the CloudWatch log group.
        # ---------------------------------------------------------------
        print(f'Received event: {event}')

        uplink = event.get("uplink")
        if uplink is None:
            print("Unsupported request received {}".format(event))
            return {
                'statusCode': 400,
                'body': json.dumps('Unsupported request received. Only uplink and notification are supported')
            }

        # ---------------------------------------------
        # Read the metadata
        # ---------------------------------------------
        wireless_metadata  = uplink.get("WirelessMetadata")
        wireless_device_id = uplink.get("WirelessDeviceId")
        sidewalk           = wireless_metadata.get("Sidewalk")
        timestamp          = sidewalk.get("Timestamp")
        data               = uplink.get("PayloadData")

        # ---------------------------------------------
        # Transform Base64-encoded Sidewalk payload into a hex string
        # ---------------------------------------------
        data_bytes   = data.encode('ascii')
        decoded_data = base64.b64decode(data_bytes).decode('ascii')
        print('decoded_data:', decoded_data)

        # ---------------------------------------------
        # Republish the original uplink to the MQTT broker
        # ---------------------------------------------
        mqtt_client = boto3.client('iot-data')
        mqtt_client.publish(
            topic=f'{geolocation_mqtt_topic_base}/{wireless_device_id}/raw',
            qos=1,
            payload=json.dumps(uplink)
        )

        # ---------------------------------------------
        # Process data fragments assembly
        # ---------------------------------------------
        fragments_assembler = FragmentsAssembler(wireless_device_id)
        full_uplink_payload = fragments_assembler.process_fragment(decoded_data, timestamp)
        if full_uplink_payload:
            # ---------------------------------------------
            # Parse TLV records inside the payload
            # ---------------------------------------------
            geolocation_scan_records = TlvParser.parse(full_uplink_payload)

            if geolocation_scan_records:
                print(f'Geolocation scan records from device: {geolocation_scan_records}')

                resolved_position = {}

                # Process WiFi scans
                if 'wifi_scans' in geolocation_scan_records:
                    # Map each WiFi scan to a WiFiAccessPoint
                    wifi_scans = list(map(lambda scan: {
                        'MacAddress': scan['mac_address'],
                        'Rss': scan['rssi']
                    }, geolocation_scan_records['wifi_scans']))

                    # Resolve position using WiFi scans
                    resolved_wifi_position = try_resolve_wifi_position(wifi_scans)
                    if resolved_wifi_position is not None:
                        # Successfully resolved WiFi position
                        print(f'WiFi position successfully resolved: {resolved_wifi_position}')
                        resolved_position['wifi'] = resolved_wifi_position
                    else:
                        print(f'WiFi position cannot be resolved')
                else:
                    print(f'No WiFi scan results in uplink')

                # Process GNSS scans
                if 'gnss_scans' in geolocation_scan_records:
                    resolved_position['gnss']=[]

                    # Resolve every NAV3 record
                    for item in geolocation_scan_records['gnss_scans']:
                        # Map each GNSS scan to a PositionEstimate
                        gnss_scan = {
                            'Payload': item['nav3_payload']
                        }

                        # Scan timestamp is taken into account if is better than 16 seconds
                        if item['time_accuracy'] < __SEMTECH_GNSS_RESOLVER_CUTOFF_TIME_ACCURACY:
                            gnss_scan['CaptureTime'] = float(item['timestamp'])
                            gnss_scan['CaptureTimeAccuracy'] = float(item['time_accuracy']) / 1000.0  # Convert ms to seconds

                        # Add aiding position if available
                        if 'assist_position' in item:
                            gnss_scan['AssistPosition'] = item['assist_position']
                        else:
                            # Enrich GNSS scan data with assisted position derived from WiFi if available
                            if 'wifi' in resolved_position and 'coordinates' in resolved_position['wifi'] and len(resolved_position['wifi']['coordinates']) >= 2:
                                gnss_scan['AssistPosition'] = [resolved_position['wifi']['coordinates'][1], resolved_position['wifi']['coordinates'][0]]

                        # Provide debug output
                        print(f'gnss_scan_3D: {gnss_scan}')

                        # Resolve position using NAV3 data and 3D solver
                        resolved_gnss_position = try_resolve_gnss_position(gnss_scan)

                        if resolved_gnss_position is not None:
                            # Successfully resolved GNSS position using 3D solver
                            print(f'GNSS position successfully resolved using 3D solver: {resolved_gnss_position}')
                            resolved_position['gnss'].append(resolved_gnss_position)
                        else:
                            # Failed to resolve GNSS position, fall back to 2D solver and try again
                            print(f'Failed to resolve GNSS position using 3D sover, falling back to 2D solver')
                            gnss_scan['Use2DSolver'] = True

                            if 'AssistAltitude' not in gnss_scan:
                                if 'wifi' in resolved_position and 'coordinates' in resolved_position['wifi'] and len(resolved_position['wifi']['coordinates']) == 3:
                                    # Add assist altitude from WiFi scan
                                    gnss_scan['AssistAltitude'] = float(resolved_position['wifi']['coordinates'][2])
                                else:
                                    # Assist altitude value is not available, use substitution value
                                    gnss_scan['AssistAltitude'] = 0.0

                            # Provide debug output
                            print(f'gnss_scan_2D: {gnss_scan}')

                            # Resolve position using NAV3 data and 2D solver
                            resolved_gnss_position = try_resolve_gnss_position(gnss_scan)
                            if resolved_gnss_position is not None:
                                # Successfully resolved GNSS position using 3D solver
                                print(f'GNSS position successfully resolved using 2D solver: {resolved_gnss_position}')
                                resolved_position['gnss'].append(resolved_gnss_position)
                            else:
                                print(f'GNSS position cannot be resolved for the given NAV3 payload. Probably sky conditions are suboptimal')

                    # Clean up the resolved position object
                    if len(resolved_position['gnss']) == 0:
                        del resolved_position['gnss']
                else:
                    print(f'No GNSS scan results in uplink')

                # Publish resolved position to the MQTT broker if it is not empy
                if resolved_position:
                    print(f'Final resolved position: {resolved_position}')
                    mqtt_client.publish(
                        topic=f'{geolocation_mqtt_topic_base}/{wireless_device_id}/position',
                        qos=1,
                        payload=json.dumps(resolved_position)
                    )
                else:
                    print(f'No resolved position available')

        return {
            'statusCode': 200,
            'body': json.dumps('Sidewalk Geolocation processing done')
        }

    except Exception:
        print(f'Unexpected error occurred: {traceback.format_exc()}')
        return {
            'statusCode': 500,
            'body': json.dumps('Unexpected error occurred: ' + traceback.format_exc())
        }
