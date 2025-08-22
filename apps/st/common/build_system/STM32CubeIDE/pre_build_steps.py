#!/usr/bin/env python3

#
##############################################################################
# file:    pre_build_step.py
# brief:   Pre-build step actions for STM32CubeIDE-based Sidewalk projects
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


import argparse
import datetime
import os
import re
import sys
import subprocess


SCRIPT_ROOT = os.path.abspath(os.path.dirname(__file__))
VERSION_HEADER_FILE_NAME = 'sid_app_version_cubeide.h'


class VersionNotFoundException(Exception):
    pass


def normalize_path(path):
    """Convert path to absolute and normalize slashes (always use forward slashes for GCC)."""
    return os.path.abspath(path).replace('\\', '/')


def find_package_root_dir(project_path: str, verbose: bool=False):
    """Try to get the git root directory starting from 'path' (or cwd if None).
    Returns the path or None if not a git repo or git not installed."""
    root_dir = None
    try:
        if verbose:
            print(f'[INFO] Trying to resolve package root path via git')
        cmd = ['git', 'rev-parse', '--show-toplevel']
        # Run in the specified directory or cwd
        result = subprocess.run(
            cmd,
            cwd=project_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
            text=True
        )
        root_dir = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        if verbose:
            print(f'[WARN] Unable to resolve package root path via git, falling back to analyzing project path')
        root_dir = normalize_path(os.path.join(project_path, '../../../..'))

    if verbose:
        print(f'[INFO] Package root  : {root_dir}')
    return root_dir


def parse_package_version(version_file_path: str, verbose: bool=False):
    try:
        version = None
        pattern = re.compile(
            r'_APPS_PACKAGE_VER\s*:=\s*'  # Key and assignment
            r'(\d+)\.(\d+)\.(\d+)'        # major.minor.patch (mandatory)
            r'(?:\.(\d+))?'               # optional .build (non-capturing group with capture inside)
        )

        with open(version_file_path, 'r') as f:
            for line in f:
                match = pattern.search(line)
                if match:
                    major, minor, patch, build = match.groups()
                    version = {
                        'major': int(major),
                        'minor': int(minor),
                        'patch': int(patch),
                        'build': int(build) if build is not None else None
                    }
        if version is None:
            raise VersionNotFoundException(f'Can\'t extract app version information from {version_file_path}')

        if verbose:
            print(f'[INFO] Identified app version: {version["major"]}.{version["minor"]}.{version["patch"]}' + (f'.{version["build"]}' if version["build"] is not None else ''))

        return version
    except Exception as e:
        print(f'[ERROR] Failed to write output file: {e}', file=sys.stderr)
        sys.exit(-2)


def get_git_commit_info(app_root_dir, verbose: bool=False):
    """
    Returns (commit_hash, commit_description) from a Git repo at app_root_dir.
    Annotates hash with (dirty) or (unknown state) if needed.
    """
    if verbose:
        print("[INFO] Retrieving branch and commit hash info from Git...")

    def run_git_command(args):
        try:
            result = subprocess.run(
                ['git'] + args,
                cwd=app_root_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            return result.returncode, result.stdout.strip(), result.stderr.strip()
        except FileNotFoundError:
            return -1, '', 'git not found'
        except Exception as e:
            return -1, '', str(e)

    # Get commit hash
    code, commit_hash, err = run_git_command(['rev-parse', 'HEAD'])
    if code != 0:
        print(f'[WARN]: Failed to get Git commit hash. Exit code: {code}. Error: {err}')
        return 'N/A', 'N/A'

    # Check for dirty working tree
    diff_code, diff_output, diff_err = run_git_command(['diff', '--stat', 'HEAD'])
    if diff_code == 0:
        if diff_output:
            commit_hash += ' (dirty)'
    else:
        print(f'[WARN]: Failed to get Git diff. Exit code: {diff_code}. Error: {diff_err}')
        commit_hash += ' (unknown state)'

    # Get commit description
    desc_code, description, desc_err = run_git_command(['describe', '--abbrev=8', '--tags'])
    if desc_code != 0:
        print(f'[WARN]: Failed to get Git description. Exit code: {desc_code}. Error: {desc_err}')
        description = 'N/A'

    return commit_hash, description


def generate_gcc_flags(package_root, build_dir, verbose: bool=False):
    try:
        output_path = os.path.join(build_dir, 'gcc_compiler_flags.txt')
        with open(output_path, 'w') as f:
            f.write(f'-fmacro-prefix-map=\"{package_root}\"=\"\"\n')

        if verbose:
            print(f'[INFO] Compiler flags written to: {output_path}')
    except Exception as e:
        print(f"[ERROR] Failed to write compiler flags file: {e}", file=sys.stderr)
        sys.exit(-3)


def generate_version_info_header(output_dir: str, version_info: dict, verbose: bool=False):
    try:
        template_path = os.path.join(SCRIPT_ROOT, f'{VERSION_HEADER_FILE_NAME}.template')
        output_path = os.path.join(output_dir, VERSION_HEADER_FILE_NAME)

        with open(template_path, 'r') as fin:
            content = fin.read()

        # Compile version string
        version_string = f'{version_info["major"]}.{version_info["minor"]}.{version_info["patch"]}' + (f'.{version_info["build"]}' if version_info["build"] is not None else '')
        build_number_definition_str:str = f'\n#define SID_APP_PROJECT_BUILD_VERSION      {version_info["build"]}\n' if version_info['build'] is not None else ''

        # Try to get commit hash and description from git
        commit_hash, commit_desc = get_git_commit_info(output_dir, verbose)

        template_params = {
            'CURRENT_YEAR'                                 : str(datetime.datetime.now().year),
            'PROJECT_VERSION'                              : version_string,
            'PROJECT_VERSION_MAJOR'                        : version_info['major'],
            'PROJECT_VERSION_MINOR'                        : version_info['minor'],
            'PROJECT_VERSION_PATCH'                        : version_info['patch'],
            'OPTIONAL_DEFINE_SID_APP_PROJECT_BUILD_VERSION': build_number_definition_str,
            'PROJECT_COMMIT_HASH'                          : commit_hash,
            'PROJECT_COMMIT_DESCRIPTION'                   : commit_desc,
        }

        for key, val in template_params.items():
            placeholder = f'@{key}@'
            content = content.replace(placeholder, str(val))

        with open(output_path, 'w') as fout:
            fout.write(content)

        if verbose:
            print(f'[INFO] Version header file written to: {output_path}')
    except Exception as e:
        print(f"[ERROR] Failed to write version header file: {e}", file=sys.stderr)
        sys.exit(-4)


def main():
    print(f'[INFO] Running pre-build actions...')
    sys.stdout.flush()

    parser = argparse.ArgumentParser(
        description='Shared pre-build steps for STM32 platform'
    )

    parser.add_argument(
        '-a', '--app-dir',
        required=True,
        help='Path to the Sidewalk application root directory (e.g., sid_900, sid_dut, etc.).'
    )

    parser.add_argument(
        '-p', '--project-dir',
        help='Path to the CubeIDE project directory (where .project/.cproject live).'
    )

    parser.add_argument(
        '-b', '--build-dir',
        default=os.getcwd(),
        help='Current build working directory (where the output file should go). Current working dir will be used if nt specified explicitly'
    )

    parser.add_argument(
        '-v', '--version-file',
        required=True,
        help='Path to the file containing application package version'
    )

    parser.add_argument(
        '--version-output-dir',
        default=None,
        help='Output directory for the version header file. If skipped, the header file will be put in the Core/Inc folder of the project'
    )

    parser.add_argument(
        '-V', '--verbose',
        action='store_true',
        help='Enable verbose output for debugging.'
    )

    args = parser.parse_args()

    try:
        # Collect essential path args
        app_root_path = normalize_path(args.app_dir)
        project_path = normalize_path(args.project_dir) if args.project_dir else None
        build_dir = normalize_path(args.build_dir)
        version_file = normalize_path(args.version_file)
        version_header_out_dir = normalize_path(args.version_output_dir if args.version_output_dir is not None else os.path.join(app_root_path, 'Core', 'Inc'))

        if args.verbose:
            print(f'[INFO] App root      : {app_root_path}')
            print(f'[INFO] Project path  : {project_path}')
            print(f'[INFO] Build dir     : {build_dir}')

        # Locate package root
        package_root = find_package_root_dir(app_root_path, args.verbose)

        # Determine app version
        version_info = parse_package_version(version_file, args.verbose)

        # Step 1 - generate project- and location-specific GCC command line flags
        generate_gcc_flags(package_root, build_dir, args.verbose)

        # Step 2 - generate header file with version information
        generate_version_info_header(version_header_out_dir, version_info, args.verbose)

        # Done
        print(f'[INFO] Pre-build actions completed successfully')
    except Exception as e:
        print(f'Pre-build script failed: {e}')
        exit(-1)


if __name__ == "__main__":
    if sys.version_info < (3, 6):
        sys.exit("Error: Python 3.6 or higher is required to run this script.")
    main()
