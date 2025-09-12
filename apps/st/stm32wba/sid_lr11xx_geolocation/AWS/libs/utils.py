# Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
##############################################################################
# file:    utils.py
# brief:   Commonly used utility functions
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

"""
Utility functions
"""

import os
import zipfile

from enum import Enum
from pathlib import Path
from colorama import init, Fore


class ErrCode(Enum):
    """Error codes"""
    EXCEPTION = 1
    CANCELLED_BY_USER = 2


_log_wait_count = 0


def log_wait():
    """Formats and prints a wait line and resets the cursor to the start of the line."""
    global _log_wait_count
    _log_wait_count += 1
    if _log_wait_count > 3: log_clear()
    print(Fore.CYAN + f' {"." * _log_wait_count}\r' + Fore.RESET, end='')


def log_clear():
    """Clears the contents of the current line"""
    global _log_wait_count
    _log_wait_count = 0
    LINE_CLEAR = '\x1b[2K'
    print(end=LINE_CLEAR)


def log_error(message):
    """Formats and prints error message."""
    log_clear()
    print(Fore.RED + f'[ERROR]  \t{message}' + Fore.RESET)


def log_warn(message):
    """Formats and prints warn message."""
    log_clear()
    print(Fore.LIGHTYELLOW_EX + f'[WARNING]  \t{message}' + Fore.RESET)


def log_info(message):
    """Formats and prints info message."""
    log_clear()
    print(f'[INFO]   \t{message}')


def log_progress(message):
    """Formats and prints progress message."""
    log_clear()
    print(Fore.CYAN + f'[PROGRESS]\t{message}' + Fore.RESET)


def log_check(message):
    """Formats and prints progress message."""
    log_clear()
    print(Fore.CYAN + f'[CHECK]\t{message}' + Fore.RESET)


def log_success(message):
    """Formats and prints success message."""
    log_clear()
    print(Fore.GREEN + f'[SUCCESS]\t{message}' + Fore.RESET)


def confirm():
    """Asks user to confirm an action."""
    log_clear()
    print(Fore.LIGHTBLUE_EX + f'[CONFIRM]  \tSend \'y\' to confirm: ' + Fore.RESET, end='')
    response = input()
    if response not in ['y', 'Y', 'yes', 'Yes']:
        terminate('Action cancelled by the user.', ErrCode.CANCELLED_BY_USER)


def eval_client_response(response: dict, success_message: str=None):
    """Evaluates AWS client response based on its status code.
    Prints result of the evaluation.
    Terminates in case of error.

    :param response:        AWS client response dict.
    :param success_message: Message to be printed in case of success.
    """
    code = response['ResponseMetadata']['HTTPStatusCode']
    if 200 <= code < 300:
        if success_message:
            log_success(success_message)
    else:
        terminate(f'Unexpected status code: {code}. Message: {response.content}', ErrCode.EXCEPTION)


def terminate(msg: str, status_code: ErrCode):
    """Prints message and terminates the execution.

    :param msg:         Message to be printed.
    :param status_code: Exit code to be returned.
    :return:
    """
    log_error(f'Terminating: {msg}')
    quit(code=status_code.value)


def read_file(file_path: Path) -> str:
    """Reads file from the given path.

    :param file_path:   Path to the file.
    :return:            String with the contents of the file.
    """
    log_info(f'Reading {file_path.name} file...')
    try:
        with open(file_path, 'r') as fd:
            content = fd.read()
        log_success("File read successfully.")
        return content
    except FileNotFoundError:
        terminate(f'{file_path.name} file does not exist.', ErrCode.EXCEPTION)


def zip_top_level_files(path, buf):
    """
    Zips files from top level of path to buffer. Omits any files starting with "test_".
    :param path:    Path to the folder to be zipped.
    :param buf:     Buffer into which the content will be inserted.
    """
    with zipfile.ZipFile(buf, 'a') as buffer_zip:
        for item in os.listdir(path):
            filepath = os.path.join(path, item)
            if os.path.isfile(filepath) and not item.startswith("test_"):
                buffer_zip.write(filepath,
                                 os.path.join("", item))


def zip_dir(path: Path, base_dir_name: str, buf):
    """
    Zips files where base_dir_name will be zipped root directory."

    :param path:            Path to the folder to be zipped.
    :param base_dir_name:   Name of directory which will be zipped root directory.
    :param buf:             Buffer into which the content will be inserted.
    """
    with zipfile.ZipFile(buf, 'a') as buffer_zip:
        for root, dirs, files in os.walk(path):
            root_split = root.split(base_dir_name)
            if len(root_split) > 1:
                zip_file_root = root_split[1]
            else:
                zip_file_root = ""
            for file in files:
                buffer_zip.write(os.path.join(root, file),
                                 os.path.join(zip_file_root, file))
# Init colorama
init()
