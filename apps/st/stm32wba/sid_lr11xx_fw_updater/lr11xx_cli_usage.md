# LR11xx Transceiver Firmware Update CLI Documentation

### Overview
This document provides a brief description of the available CLI commands to perform LR11xx firmware updates. This project contains multiple build configurations targeting LR1110, LR1120, and LR1121 transceivers specifically. The only difference between these build configurations is the collection of LR11xx firmware images that are included into the STM32WBA5x binary, the CLI command set is identical.

### Firmware Types
LR11xx transceivers generally support two types of firmware:

- Transceiver Mode (**TRX**)
- LoRa Modem-E (**Modem-E**)

A TRX firmware allows direct control over the radio PHY, while Modem-E firmware includes additional layers that allow to run LoRaWAN stack directly on the LR11xx. Modem-E firmware image comes in two formats (V1 and V2), depending on the target LR11xx IC (V1 applies to LR1110, V2 applies to LR1120). Sidewalk stack requires a TRX firmware to run properly, it is not compatible with the Modem-E firmware. Additionally, ST oficially supports only the following LR11xx firmware versions for Sidewalk applications:

- **LR1110**: `04.01`
- **LR1120**: `02.01`
- **LR1121**: `01.03`

The older versions are not supported and may or may not work properly. While the ST's LR11xx Sidewalk HAL generally allows startup with the outdated firmware version, this does not mean the user may expect stable and reliable operation. The startup is allowed to provide firmware update features only. Please always update LR11xx to the specified supported version. 

### Operating Principles
This app provides both a fully automated update process that requires no user intercations (except initiating the process itself) and the individual CLI commands for every potential step of LR11xx firmware (re)flashing. Normally an automated update procedure should be used, but if any deviations from the reference flow are required you are free to use the individual step-by-step CLI commands.

The automated update process is designed in a way that allows to (re)flash the LR11xx from virtually any state (e.g. erased flash, invalid firmware uploaded, LoRa Modem-E firmware is installed, etc.).

### CLI commands usage and format
Arguments to the command are shown in <>
```
    reset                                       - initiate host MCU reset (not the resert of LR11xx IC only)

    lr11xx init                                 - initialize LR11xx driver and underlying peripherals (e.g. SPI, GPIO)
                                                    This command invokes sid_pal_radio_init() method and tries to detect the attached LR11xx device

    lr11xx deinit                               - deinitialize LR11xx driver and release all the associated hardware and software resources

    lr11xx reinit                               - sequentially perform deinit and init operations for LR11xx driver
                                                    This can be useful to reset not only LR11xx but associated MCU resources as well

    lr11xx run_mode <mode>                      - get or set LR11xx run mode (bootloader or app)
                                                    No <mode> specified: reads out current run mode from LR11xx
                                                    
                                                    <mode> specified: switches LR11xx to the desired mode. Valid values: app, bootloader

                                                    Examples:
                                                        lr11xx run_mode
                                                        lr11xx run_mode bootloader
                                                        lr11xx run_mode app

    lr11xx version                              - read out firmware and hardware version information from LR11xx device.
                                                    In bootloader mode the bootloader version will be reported

    lr11xx fw auto_update <version> <--force>   - perform automated update procedure
                                                    This mode automatically performs a full update cycle and requires no user interaction.

                                                    <version> parameter is optional and can be specified in any format (e.g. 0x0401, 0401, 04.01). 
                                                    Additionally "factory" and "latest" can be specified as target version
                                                    If no version is specified the latest avaialble image will be installed.

                                                    <--force> or <-f> flag is optional. If not present, reflashing with the same firmware version is aborted.

                                                    Examples:
                                                        lr11xx fw auto_update
                                                        lr11xx fw auto_update 03.07
                                                        lr11xx fw auto_update factory

    lr11xx fw check_image <version>             - perform firmware image signature check using LR11xx crypto facilities
                                                    <version> is a mandatory parameter, it must specify one of the available firmware images.

                                                    The check is performed by LR11xx IC itself and requries certain firwmare version
                                                    to be running on the device (e.g. 03.08 for LR1110 and 01.02 for LR112x).

                                                    This command is only valid when LR11xx is in app mode

    lr11xx fw erase                             - erase LR11xx flash
                                                    This command is only valid when LR11xx is in bootloader mode

    lr11xx fw flash <version>                   - flash the selected LR11xx firmwareimage
                                                    <version> is a mandatory parameter, it must specify one of the available firmware images.

                                                    This command is only valid when LR11xx is in bootloader mode

                                                    Flash must be erased before writing to it

    lr11xx fw list                              - display the list of avaialble LR11xx firmware images
```

### Typical Workflow for an Automated Update
```
lr11xx init
lr11xx fw auto_update
...
lr11xx deinit
```

### Typical Workflow for a Manual Update
```
lr11xx init

lr11xx run_mode app
lr11xx fw check_image latest

lr11xx run_mode bootloader
lr11xx fw erase
lr11xx fw flash latest

lr11xx run_mode app
lr11xx version
...
lr11xx deinit
```
