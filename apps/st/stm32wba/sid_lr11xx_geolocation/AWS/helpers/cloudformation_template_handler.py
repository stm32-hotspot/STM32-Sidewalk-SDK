#
##############################################################################
# file:    cloudformation_template_handler.py
# brief:   Helper class to manage CloudFormation template uploads and deletion
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
from io import BytesIO
from os import path

from constants.geolocation_demo_constants import GeolocationDemoConstants
from libs.s3_client import S3Client
from libs.utils import *


class CloudFormationTemplateHandler:

    @staticmethod
    def upload_template(s3_client: S3Client, lambda_sources: dict):
        constants = GeolocationDemoConstants()

        if not s3_client.bucket_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME):
            s3_client.create_bucket(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME)

        s3_client.put_object(
            bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME,
            bucket_key=constants.CLOUDFORMATION_TEMPLATE_DST_FILENAME,
            file_path=str(path.join(constants.CLOUDFORMATION_TEMPLATE_SRC_ROOT_DIR, constants.CLOUDFORMATION_TEMPLATE_SRC_FILENAME)),
            content_type='text/yaml'
        )

        for lambda_name, src_folder in lambda_sources.items():
            buffer = BytesIO()

            log_info(f'Compressing {lambda_name} files...')
            zip_top_level_files(path.join(constants.LAMBDA_SRC_ROOT_DIR, src_folder), buffer)
            buffer.seek(0)

            log_info(f'Uploading {lambda_name} files to S3 as {src_folder}.zip...')
            s3_client.put_object(
                bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME,
                bucket_key=f'{src_folder}.zip',
                raw_bytes=buffer,
                content_type='application/zip'
            )


    @staticmethod
    def delete_template(s3_client: S3Client, lambda_src_names=None):
        constants = GeolocationDemoConstants()

        if s3_client.bucket_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME):
            # Delete the CloudFormation associated with this app from the S3 bucket
            if s3_client.object_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=constants.CLOUDFORMATION_TEMPLATE_DST_FILENAME):
                s3_client.delete_object(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=constants.CLOUDFORMATION_TEMPLATE_DST_FILENAME)

            for file_name in lambda_src_names:
                if s3_client.object_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=f'{file_name}.zip'):
                    log_info(f'Deleting Lambda source file {file_name}.zip from S3')
                    s3_client.delete_object(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=f'{file_name}.zip')

            # Delete the bucket itself if it is empty
            if s3_client.is_bucket_empty(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME):
                s3_client.delete_bucket(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME)
