# Sidewalk Provisioning Script

This script can be used to create a 'manufacturing page' in flash which can be
flashed on the device with the correct Sidewalk identity information and keys.

A device with correctly setup is considered to be provisioned. After
provisioning there are further steps required to register the device on the
Sidewalk network.


## Script formatting

The script arguments have the following format

```sh
sid_provision <PLATFORM_NAME> <INPUT_TYPE> arguments
```

Currently provision script supports 1 platform

 * _st_

and supports 3 input_types

 * _aws_
 * _acs_console_
 * _bb_

_acs_ and _bb_ are on path to be deprecated

Depending on the *PLATFORM_NAME* and *INPUT_TYPE*, the arguments required will differ since each platform supports different types of outputs and miscellaneous input options

## Create manufacturing page by ACS console

Information Required

- Acs Console JSON file download
- Application server public key, which is generated from the
  application_server_cert/generate_application_server.py script (see
  application_server_cert/readme.md)
- SIDEWALK_ID which is embedded in the name of the json file download from ACS
  eg certificate_0123456789.json SIDEWALK_ID=0123456789

### For ST devices (STM32WBA family)

The script supports STM32WBA devices with 512KB, 1MB, and 2MB of flash. Use `--chip` parameter to specify the exact MCU you are targeting. The value for the `--chip` parameter is generally defined as `WBA{model number}x{flash size code}`, e.g. `WBA65xI`, `WBA55xG`, `WBA52xG`, etc. If `--chip` parameter is not specified explicitly, the default selection is `WBA65xI`. Usage example:

```sh
SIDEWALK_ID=0123456789
sid_provision st acs --chip WBA55xG --json certificate_${SIDEWALK_ID}.json \
 --app_srv_pub app-server-ed25519.public.bin
```
On success the `st_acs_{chip selection}.bin` and `st_acs_{chip selection}.hex` files containing the manufacturing data are generated.

### The app_srv_pub key can also be given as a 32 byte hex string

```
APP_SRV_PUB_HEX=0123456789012345678901234567890123456789012345678901234567890123
SIDEWALK_ID=0123456789
sid_provision acs --json certificate_${SIDEWALK_ID}.json \
 --app_srv_pub ${APP_SRV_PUB_HEX} \
 --config ${MFG_PAGE_CONFIG} --output_bin mfg.bin
```

## Create manufacturing page by AWS cli

Information Required

- JSON response of `aws iotwireless get-device-profile .... > device_profile.json` response saved to device_profile.json
- JSON response of `aws iotwireless get-wireless-device .... > wiresless_device.json` response saved to wireless_device.json

### For ST devices (STM32WBA family)

The script supports STM32WBA devices with 512KB, 1MB, and 2MB of flash. Use `--chip` parameter to specify the exact MCU you are targeting. The value for the `--chip` parameter is generally defined as `WBA{model number}x{flash size code}`, e.g. `WBA65xI`, `WBA55xG`, `WBA52xG`, etc. If `--chip` parameter is not specified explicitly, the default selection is `WBA65xI`. Usage example:

```sh
sid_provision st aws --chip WBA55xG --wireless_device_json wireless_device.json --device_profile_json device_profile.json
```
On success the `st_aws_{chip selection}.bin` and `st_aws_{chip selection}.hex` files containing the manufacturing data are generated.

# Create manufacturing page by Black Box JSON

Used if connecting to a Black Box server to register devices during manufacturing

Information Required

- JSON response from the Black Box server, saved to a file
- Application server public key, which is generated from the
  application_server_cert/generate_application_server.py script (see
  application_server_cert/readme.md)

```
sid_provision bb --config ${MFG_PAGE_CONFIG} --json bb_response.json \
        --output_bin mfg.bin \
        --app_srv_pub app-server-ed25519.public.bin
```

---
**NOTE**

If the user gives the `output_bin` argument and  the bin file indicated by `output_bin` contains prefilled binary, then only the
data offsets indicated by platform `config.yaml` are overwritten, the rest of the binary
file is left as is. This allows for users to merge sidewalk provision data and
their own custom mfg data.

To see the default `config.yaml` for the platform run the script with help argument

```sh
sid_provision st aws -h
usage: sid_provision st aws [-h] [--wireless_device_json WIRELESS_DEVICE_JSON] [--device_profile_json DEVICE_PROFILE_JSON]
                                 [--certificate_json CERTIFICATE_JSON] [--chip {WBA65xI}] [--dump_raw_values] [--config CONFIG]
                                 [--output_bin OUTPUT_BIN] [--output_hex OUTPUT_HEX]

options:
  -h, --help            show this help message and exit
  --wireless_device_json WIRELESS_DEVICE_JSON
                        Json Response of 'aws iotwireless get-wireless-device'
  --device_profile_json DEVICE_PROFILE_JSON
                        Json response of 'aws iotwireless get-device-profile ...'
  --certificate_json CERTIFICATE_JSON
                        Certificate json generated from sidewalk aws console
  --chip {WBA65xI}     Which chip to generate the mfg page (default: WBA65xI)
  --dump_raw_values     Dump the raw values for debugging
  --config CONFIG       Config Yaml that defines the mfg page offsets (default: <PATH_TO_CONFIG_FILE>)
  --output_bin OUTPUT_BIN
                        Output bin file, if this file does not exist - it will be created, if it does exist the data at - the offsets
                        defined in the config file will be - overwritten by provision data (default: <PATH_TO_OUTPUT_BIN>)
  --output_hex OUTPUT_HEX
                        Output hex file, default chip offset is used when generating hexfile  (default: <PATH_TO_OUTPUT_HEX>)
```

The default config file is indicated by --config CONFIG    < Help> default: <PATH TO FILE>
