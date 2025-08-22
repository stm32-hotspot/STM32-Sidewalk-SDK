# STM32WBA5x Geolocation Sample App

This is a sample application demonstrating geolocation capabilities of Semtech’s LR11xx transceivers (LR1110 and LR1120) running on **STM32WBA5x microcontrollers** based on **STMicroelectronics' Amazon Sidewalk stack**. The demo showcases WiFi-based and GNSS-based geolocation and consists of two main components:

1. **Embedded Device Firmware** – Runs on an STM32WBA5x microcontroller and interfaces with an LR11xx transceiver for geolocation scanning and Sidewalk communication.
2. **AWS Cloud Backend** – Processes geolocation scan results from the device, reconstructs fragmented uplink messages, resolves location data via AWS Location Services, and publishes device positions via MQTT.

## Features
### Embedded Device Firmware
- Supports Amazon Sidewalk-over-BLE and Sidewalk-over-FSK for initial device registration.
- Uses Sidewalk-over-FSK or Sidewalk-over-LoRa for normal device-to-cloud and cloud-to-device communication.
- Periodically performs geolocation scans:
  - WiFi sniffing (scans nearby WiFi APs for MAC addresses and RSSI values).
  - GNSS scan (captures satellite signals for precise positioning).
  - Hybrid mode (WiFi + GNSS).
- Serializes collected geolocation data, fragments messages to fit Sidewalk MTU limits, and sends them to the cloud.
- Implements efficient power management to optimize battery life.

### AWS Cloud Backend
- Listens for incoming geolocation data via **AWS IoT Core destination** for Sidewalk devices.
- **AWS IoT Rule** forwards data to an **AWS Lambda function**.
- Lambda temporarily stores message fragments in **DynamoDB** until all parts arrive.
- Lambda reassembles the complete message, extracts **WiFi and GNSS scan data**.
- Calls **AWS IoT Core Device Location** to resolve the device's location.
- Publishes the calculated position to an **MQTT topic** for data visualization.

~~~text
+---------------------+       +-------------------------+       +--------------------------+
| STM32WBA5x + LR11xx | ----> | AWS IoT Core (Sidewalk) | ----> |  AWS Lambda (Processing) |
| (WiFi/GNSS scans)   |       |                         |       |  - Reassemble fragments  |
| Amazon Sidewalk     |       | AWS IoT Rule            |       |  - Extract scan data     |
| communication       |       |                         |       |  - Call AWS Location     |
+---------------------+       +-------------------------+       +--------------------------+
                                                                              |
                                                                              v
                                                            +------------------------------+
                                                            | AWS IoT Core Device Location |
                                                            | - Resolve WiFi/GNSS          |
                                                            | - Calculate coordinates      |
                                                            +------------------------------+
                                                                              |
                                                                              v
                                                            +------------------------------+
                                                            | AWS IoT Core (MQTT)          |
                                                            | - Publish device position    |
                                                            +------------------------------+

~~~

## Prerequisites
- **LR1110 should run on firmware 04.01 and LR1120 - on 01.02**. If your LR11xx runs a different version you can use `sid_lr11xx_fw_updater` app to easily reflash your LR11xx with the latest firmware
- Download and install Python 3.9 or above (https://www.python.org/)
- Create an AWS account (https://aws.amazon.com/)
- Set up an AWS user and its credentials:
  - create user in AWS IAM service ([Creating IAM user](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_users_create.html#id_users_create_console))
  - configure user's authentication credentials ([Managing access keys -> To create an access key](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_credentials_access-keys.html#Using_CreateAccessKey))
  - add user permissions to create resources:
    - if your user has Admin permissions, prerequisite is already satisfied, you can skip this point
    - otherwise you need to assign your user a policy with proper permissions
      - go to the IAM console, create the policy
      - assign created policy to your user

      Refer to the [IAM tutorial: Create and attach your first customer managed policy](https://docs.aws.amazon.com/IAM/latest/UserGuide/tutorial_managed-policies.html) for further guidance.

## Getting Started
### 1. Install virtual environment

1. Open command line terminal and navigate to project's `./AWS` directory. You man need to run CMD as Administrator on Windows environments.

2. Install virtualenv and required packages. Just copy/paste commands to the terminal.
   You may need to use either `python` or `python3` alias, depending on your configuration.

- **Linux / MacOS**:
```
python3 -m pip install --user virtualenv
python3 -m venv geolocation-app-env
source geolocation-app-env/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

- **Windows**:
```
python -m pip install --user virtualenv
python -m venv geolocation-app-env
geolocation-app-env\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### 2. Fill out configuration file

Fill out the [config](./AWS/config.yaml) (./AWS/config.yaml) file with your details (or leave default values). 

| field                       | default value                         | description
| ---                         | ---                                   | ---
| *AWS_PROFILE*               | *null*                                | Profile to be used during the stack creation. If you have a custom named profile in your AWS CLI configuration files, replace 'null' with the name of your profile (e.g. 'default'). Usually, you'd have just one profile named 'default'. If you don't have any profiles configured you can specify AWS_ACCESS_KEY_ID
| *AWS_ACCESS_KEY_ID*         | *null* **(need to be overwritten)**   | Your AWS_ACCESS_KEY_ID. This parameter is used when *AWS_PROFILE* is null
| *AWS_SECRET_ACCESS_KEY*     | *null* **(need to be overwritten)**   | Your AWS_SECRET_ACCESS_KEY. This parameter is used when *AWS_PROFILE* is null
| *AWS_SESSION_TOKEN*         | *null* **(need to be overwritten)**   | Your AWS_SESSION_TOKEN. This parameter is used when *AWS_PROFILE* is null
| *AWS_DEFAULT_REGION*        | *null*                                | Keep it null, *us-east-1* will be forced since Sidewalk is currently limited to this region only
| *INTERACTIVE_MODE*          | *False*                               | Enables interactive mode (confirmation prompts).
| *SID_DEMO_NAME*             | *SidewalkGeolocation*                 | Name of the demo app. This name is used as prefix for the AWS resources that are deployed for this app
| *SID_DEMO_MQTT_TOPIC_ROOT*  | *sidewalk/geolocation*                | Top-level MQTT topic for publishing results. Topic scheme is *SID_DEMO_MQTT_TOPIC_ROOT/{WirelessDeviceId}/position* for resolved device location and *SID_DEMO_MQTT_TOPIC_ROOT/{WirelessDeviceId}/raw* to monitor raw uplink fragments arriving from the device
| *SID_DEMO_DEVICE_NAME*      | *null*                                | User-friendly Sidewalk device name. If this value is null the device name is set to *{SID_DEMO_NAME}Device*
| *SID_DEMO_PROVISION_DEVICE* | *True*                                | Automatically register a new Sidewalk device, assigns it to this demo app and generates manufacturing data image to be flashed to the STM32WBA5x board
| *PROVISION_OUTPUT_DIR*      | *manufacturing_data*                  | Output directory to store device provisioning information (manufacturing data)
| *PROVISION_SCRIPT_DIR*      | *path to provisioning tool in the current package* | Path tho the Sidewalk provisioning tool. Normally you don't need to edit this setting

### 3. Deploy cloud infrastructure

For the sample application to work, you need to deploy necessary resources to your AWS account.
**Before running the script, ensure that you have sufficient permissions to create resources 
(see: [Prerequisites](#Prerequisites))**.

|All the resources need to be created in *us-east-1* region. If *config* file specifies another region, it will be ignored.
|---|

|WARNING: You will be billed for the usage of AWS resources created by this application. |
|---|

    Run deployment script:
    ```
    python3 deploy_geolocation_demo_stack.py
    ```
    Type `y` when asked to proceed.
    Wait for the deployment to complete (it usually takes a few minutes).

### 4. Provision edge device

|NOTE: If you enabled *SID_DEMO_PROVISION_DEVICE* in the deployment config you can  skip this step as the device is provisioned automatically |
|---|

Refer to the generic Sidewalk documentation to manually provision a device. Normally you don't have to go through this step since deployment script provisions a device for you unless you explicitly disable this step in the configuration file

### 5. Flash edge device manufacturing data

In this step you will program binaries onto your development kit. Use STM32CubeProgrammer ato upload the generated device manufacturing data image (typically stored in `manufacturing_data/{SID_DEMO_DEVICE_NAME}.hex` file). Since `.hex` contains both data and placement information you don't have to care about the flashing address for the manufacturing data.

It's also recommended to perform full chip erase before flashing the manufacturing data image. This will ensure both the device is clean and no past data will affect the new firmware and any pre-existing firmware won't alter the manufacturing data

### 6. Build and flash the firmware

- Open STM32CubeIDE and import a relevant project from [STM32CubeIDE](./STM32CubeIDE) folder
- Select a build configuration (or keep the default selection)
- Build the selected configuration
- Flash application firmware

### 7. Enjoy the application

The device will automatically perform Sidewalk Device Registration procedure using BLE or FSK link (depending on the selected build configuration) and then it will switch to the normal link. It may take some time to receive the first scan results, especially for GNSS cold start with outdated Almanac data. You can observe arriving Sidewalk uplinks and resolved position information by subscribing to `{SID_DEMO_MQTT_TOPIC_ROOT}/+/raw` (e.g. `sidewalk/geolocation/+/raw`) and `{SID_DEMO_MQTT_TOPIC_ROOT}/+/position` (e.g. `sidewalk/geolocation/+/position`) MQTT topics respectively

## Removing the demo from the cloud

It is recommended to delete the backend resources after you've finished with this deno to avoid additional charges for AWS resource usage. To simplify the tear-down procedure you can use `deploy_geolocation_demo_stack.py` script in `AWS` folder to automatically delete all the associated cloud resources

    Run delete script:
    ```
    python3 delete_geolocation_demo_stack.py
    ```
    Type `y` when asked to proceed.
    Wait for the deletion to complete (it usually takes a few minutes).
