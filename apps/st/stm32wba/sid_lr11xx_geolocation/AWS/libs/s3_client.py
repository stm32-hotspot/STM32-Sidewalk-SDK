# Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
##############################################################################
# file:    s3_client.py
# brief:   S3 operations
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

import os
import boto3

from botocore.exceptions import ClientError
from io import BytesIO
from pathlib import Path
from libs.utils import log_info, log_success, terminate, ErrCode


class S3Client:
    """
    Provides S3 client with methods necessary to delete bucket contents.

    Attributes
    ----------
        _client: botocore.client.s3
            Client to s3.

    """

    def __init__(self, session: boto3.Session):
        self._client = session.client(service_name='s3')
        self._resource = session.resource(service_name='s3')
        self.region_name = session.region_name

    def delete_bucket_content(self, bucket_name):
        """
        Deletes contents of buckets
        :param bucket_name: name of bucket to clear
        """
        try:
            log_info(f'Deleting objects from {bucket_name} bucket...')
            paginator = self._client.get_paginator("list_objects_v2")
            results = paginator.paginate(Bucket=bucket_name).build_full_result().get("Contents", None)
            object_names = []
            if results is not None:
                for obj in results:
                    obj_dict = {"Key": obj["Key"]}
                    object_names.append(obj_dict)
            if len(object_names) > 0:
                self._client.delete_objects(Bucket=bucket_name,
                                            Delete={"Objects": object_names})
            log_success('Objects deleted, bucket is empty.')
        except ClientError as e:
            if e.response['Error']['Code'] in ['ValidationError', 'NoSuchBucket']:
                log_success(f'{bucket_name} doesn\'t exist, skipping.')
            else:
                terminate(f'Unable to delete objects from the bucket: {e}.', ErrCode.EXCEPTION)

    def put_files(self, bucket_name, build_dir):
        """
        Puts content of web application to s3 bucket
        :param bucket_name: name of s3 bucket
        :param build_dir: path to gui directory. Needs to be provided to provide compatibility with mac build
        (it takes parent as lib)
        """
        web_app = 'SensorMonitoringApp'

        try:
            self.delete_bucket_content(bucket_name)
            log_info(f'Uploading {web_app} files to the S3 bucket...')
            suffix_map = {
                '.svg': 'image/svg+xml',
                '.png': 'image/png',
                '.js': 'application/javascript',
                '.css': 'text/css',
                '.html': 'text/html',
                '.json': 'application/json',
                '.map': 'application/json',
                '.txt': 'text/plain',
                '.woff2': 'font-woff2',
                '.woff': 'application/font-woff',
                '.ttf': 'application/font-sfnt'
            }
            for root, dirs, files in os.walk(build_dir):
                for file in files:
                    if root == str(build_dir):
                        key = file
                    else:
                        key = root[len(str(build_dir)) + 1:len(root)] + '/' + file
                    file_path = Path(root, file)
                    key = key.replace('\\', '/')
                    if file_path.stem.startswith('index') and file_path.suffix == '.js':
                        with open(file_path, 'r') as fd:
                            # override template: replace <api_gw_invoke_url> placeholder with actual value
                            content = fd.read()
                            with BytesIO(bytes(content, 'utf-8')) as data:
                                self._client.put_object(Bucket=bucket_name, Key=key, Body=data, ContentType=suffix_map[file_path.suffix])
                    else:
                        with open(file_path, 'rb') as data:
                            self._client.put_object(Bucket=bucket_name, Key=key, Body=data, ContentType=suffix_map[file_path.suffix])
            log_success(f'Files uploaded successfully.')
        except ClientError as e:
            terminate(f'Unable to upload files: {e}.', ErrCode.EXCEPTION)

    def bucket_exists(self, bucket_name: str):
        try:
            self._client.head_bucket(Bucket=bucket_name)
            return True
        except self._client.exceptions.ClientError as e:
            if e.response['Error']['Code'] == '404':
                return False
            else:
                raise

    def object_exists(self, bucket_name: str, bucket_key: str):
        try:
            self._client.head_object(Bucket=bucket_name, Key=bucket_key)
            return True
        except self._client.exceptions.ClientError as e:
            if e.response['Error']['Code'] == '404':
                return False
            else:
                raise

    def create_bucket(self, bucket_name: str, object_ownership: str = None):
        if not object_ownership:
            return self._client.create_bucket(Bucket=bucket_name)
        else:
            return self._client.create_bucket(
                Bucket=bucket_name,
                ObjectOwnership=object_ownership
            )

    def put_public_access_block(
            self,
            bucket_name: str,
            block_public_acls: bool,
            ignore_public_acls: bool,
            block_public_policy: bool,
            restrict_public_buckets: bool
        ):
        return self._client.put_public_access_block(
            Bucket=bucket_name,
            PublicAccessBlockConfiguration={
                'BlockPublicAcls': block_public_acls,
                'IgnorePublicAcls': ignore_public_acls,
                'BlockPublicPolicy': block_public_policy,
                'RestrictPublicBuckets': restrict_public_buckets
            }
        )

    def put_bucket_acl(self, bucket_name: str, acl: str):
        return self._client.put_bucket_acl(Bucket=bucket_name, ACL=acl)

    def delete_bucket(self, bucket_name: str):
        return self._client.delete_bucket(
            Bucket=bucket_name
        )

    def put_object(self, bucket_name: str, bucket_key: str, content_type: str, file_path: str=None, raw_bytes: BytesIO=None):
        if file_path is not None and raw_bytes is None:
            with open(file_path, 'r') as fd:
                content = fd.read()
                with BytesIO(bytes(content, 'utf-8')) as data:
                    self._client.put_object(Bucket=bucket_name, Key=bucket_key, Body=data, ContentType=content_type)
        elif file_path is None and raw_bytes is not None:
            self._client.put_object(Bucket=bucket_name, Key=bucket_key, Body=raw_bytes, ContentType=content_type)
        else:
            raise AttributeError('You cannot specify both file_path and raw_bytes at the same time')

    def upload_fileobj(self, bucket_name: str, bucket_key: str, file_path: str):
        with open(file_path, 'rb') as data:
            self._client.upload_fileobj(data, bucket_name, bucket_key)

    def delete_object(self, bucket_name: str, bucket_key: str):
        return self._client.delete_object(
            Bucket=bucket_name,
            Key=bucket_key,
        )

    def is_bucket_folder_empty(self, bucket_name: str, folder_name: str) -> bool:
        if self.bucket_exists(bucket_name):
            bucket = self._resource.Bucket(bucket_name)
            count = bucket.objects.filter(Prefix=folder_name)
        else:
            count = 0

        return 0 == count

    def is_bucket_empty(self, bucket_name: str) -> bool:
        return self.is_bucket_folder_empty(bucket_name,'/')
