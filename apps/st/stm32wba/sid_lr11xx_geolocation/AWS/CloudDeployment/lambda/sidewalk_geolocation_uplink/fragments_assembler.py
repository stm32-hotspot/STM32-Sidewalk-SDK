#
##############################################################################
# file:    fragments_assembler.py
# brief:   Custom Sidewalk payload processor to assemble fragmented uplinks
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
import os

from dataclasses import dataclass
from datetime import datetime, timezone


@dataclass
class UplinkFragment:
    token: int
    total_fragments: int
    current_fragment: int
    payload: str


class UplinkFragmentValidationError(Exception):
    pass


class FragmentsAssembler:
    """
    Manages temporary storage of fragmented uplinks and reassembles complete app messages from fragmented data
    """

    __HEADER_TOKEN_OFFSET: int            = 0 
    __HEADER_TOKEN_SIZE: int              = 2
    __HEADER_TOTAL_FRAGMENTS_OFFSET: int  = __HEADER_TOKEN_OFFSET + __HEADER_TOKEN_SIZE
    __HEADER_TOTAL_FRAGMENTS_SIZE: int    = 1
    __HEADER_CURRENT_FRAGMENT_OFFSET: int = __HEADER_TOTAL_FRAGMENTS_OFFSET + __HEADER_TOTAL_FRAGMENTS_SIZE
    __HEADER_CURRENT_FRAGMENT_SIZE: int   = 1
    __PAYLOAD_START_OFFSET: int           = __HEADER_CURRENT_FRAGMENT_OFFSET + __HEADER_CURRENT_FRAGMENT_SIZE

    __DYNAMODB_TABLE_NAME: str            = os.environ['UPLINK_FRAGMENTS_DYNAMODB_TABLE']
    __DYNAMODB_FRAGMENT_TTL: int          = int(os.environ['UPLINK_FRAGMENT_DYNAMODB_TTL'])

    @staticmethod
    def __byte_size_to_hex_str_size(byte_size: int):
        return byte_size * 2

    def __init__(self, wireless_device_id):
        self.wireless_device_id = wireless_device_id
        self.__dynamodb_client = boto3.client('dynamodb')

    def process_fragment(self, raw_uplink_payload, timestamp):
        # Check that raw payload length is not too short
        if len(raw_uplink_payload) < FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__PAYLOAD_START_OFFSET + 1):
            # Raise validation error
            raise UplinkFragmentValidationError(f'Raw uplink fragment payload length too short: {len(raw_uplink_payload)}')

        # Build UplinkFragment object
        fragment = UplinkFragment(
            token           =int(raw_uplink_payload[FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__HEADER_TOKEN_OFFSET)           :FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__HEADER_TOKEN_OFFSET            + FragmentsAssembler.__HEADER_TOKEN_SIZE)],            16),
            total_fragments =int(raw_uplink_payload[FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__HEADER_TOTAL_FRAGMENTS_OFFSET) :FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__HEADER_TOTAL_FRAGMENTS_OFFSET  + FragmentsAssembler.__HEADER_TOTAL_FRAGMENTS_SIZE)],  16),
            current_fragment=int(raw_uplink_payload[FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__HEADER_CURRENT_FRAGMENT_OFFSET):FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__HEADER_CURRENT_FRAGMENT_OFFSET + FragmentsAssembler.__HEADER_CURRENT_FRAGMENT_SIZE)], 16),
            payload         =str(raw_uplink_payload[FragmentsAssembler.__byte_size_to_hex_str_size(FragmentsAssembler.__PAYLOAD_START_OFFSET)          :])
        )
        print(f'Uplink info: token: {fragment.token:0>4X}, fragment ID: {fragment.current_fragment}, total fragments: {fragment.total_fragments}')

        # Validate the header
        if fragment.current_fragment > fragment.total_fragments:
            # Raise validation error
            raise UplinkFragmentValidationError(f'Invalid fragment ID: {fragment.current_fragment}, total fragments: {fragment.total_fragments}')

        # Parse ISO datetiem string and convert to Unix epoch with seconds precision
        try:
            timestamp_dt = datetime.strptime(timestamp, '%Y-%m-%dT%H:%M:%S.%fZ')
        except ValueError:
            timestamp_dt = datetime.strptime(timestamp, '%Y-%m-%dT%H:%M:%SZ')
        timestamp_dt = timestamp_dt.replace(tzinfo=timezone.utc)
        timestamp_epoch = int(timestamp_dt.timestamp())

        # Process fragment and try to assemble full uplink message
        full_uplink_payload = ''

        if fragment.total_fragments > 1:
            # Query DynamoDB to get any other fragments associated with current token
            existing_fragments = self.__dynamodb_client.query(
                TableName=FragmentsAssembler.__DYNAMODB_TABLE_NAME,
                KeyConditionExpression='uplink_id = :uplink_id',
                FilterExpression='expiry > :cutoff_time', # Filter out expired fragments that may possible remain in DB from the past
                ExpressionAttributeValues={
                    ':uplink_id':          {'S': f'{self.wireless_device_id}-{fragment.token:0>4x}'},
                    ':cutoff_time':        {'N': str(timestamp_epoch - FragmentsAssembler.__DYNAMODB_FRAGMENT_TTL)}
                }
            )

            # Filter out fragments that have the same index as the current fragment - this is required to cover possible uplink duplicates
            existing_fragments = list(filter(
                lambda item: int(item['fragment_idx']['N']) != fragment.current_fragment,
                existing_fragments['Items']
            ))

            # Check if we can assemble full uplink message
            if len(existing_fragments) + 1 == fragment.total_fragments:
                print(f'All {fragment.total_fragments} fragments are available for uplink token {fragment.token:0>4X}, reassembling application payload...')

                # Add current fragment to list of existing fragments
                existing_fragments.append({
                    'uplink_id':          {'S': f'{self.wireless_device_id}-{fragment.token:0>4x}'},
                    'fragment_idx':       {'N': str(fragment.current_fragment)},
                    'timestamp':          {'N': str(timestamp_epoch)},
                    'payload':            {'S': fragment.payload}
                })

                # Sort fragments by index
                existing_fragments.sort(key=lambda item: int(item['fragment_idx']['N']))

                # Assemble full uplink message
                for item in existing_fragments:
                    full_uplink_payload += item['payload']['S']

                    # Delete processed fragment from DynamoDB
                    if item['fragment_idx']['N'] != str(fragment.current_fragment):
                        print(f'Deleting fragment {item["fragment_idx"]["N"]} from DynamoDB')
                        self.__dynamodb_client.delete_item(
                            TableName=FragmentsAssembler.__DYNAMODB_TABLE_NAME,
                            Key={
                                'uplink_id': {'S': item['uplink_id']['S']},
                                'fragment_idx': {'N': item['fragment_idx']['N']}
                            }
                        )
            else:
                # Store current fragment in DynamoDB
                print(f'{fragment.total_fragments - len(existing_fragments) - 1} fragments are still missing for uplink token {fragment.token:0>4X}. Storing fragment {fragment.current_fragment} to DynamoDB')

                self.__dynamodb_client.put_item(
                    TableName=FragmentsAssembler.__DYNAMODB_TABLE_NAME,
                    Item={
                        'uplink_id':          {'S': f'{self.wireless_device_id}-{fragment.token:0>4x}'},
                        'fragment_idx':       {'N': str(fragment.current_fragment)},
                        'timestamp':          {'N': str(timestamp_epoch)},
                        'expiry':             {'N': str(timestamp_epoch + FragmentsAssembler.__DYNAMODB_FRAGMENT_TTL)},
                        'payload':            {'S': fragment.payload}
                    }
                )
        else:
            print('Single fragment uplink, no need to store')
            full_uplink_payload = fragment.payload

        # Print full link payload if it is not empty
        if full_uplink_payload:
            print(f'Full uplink payload: {full_uplink_payload}')

        return full_uplink_payload
