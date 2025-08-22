# Sidewalk-BLE Co-existence CLI Documentation

### Overview
This document provides a brief description of the available CLI commands to test the BLE co-existence operation. The `sid_dut_ble_coexistence` app is a modified version of the standard `sid_dut` app. It keeps all the functionality of the `sid_dut` and adds additional CLI commands on top of that to run custom BLE simultaneously.

Overall, there are four modes of BLE co-existence supported by the driver:
- *Sidewalk-only* - no co-existence is possible, BLE can be used by Sidewalk stack only
- *Routing* - whenever Sidewalk stack is de-initialized it simply transparently forwards calls to the `SVCCTL_App_Notification`, `BLECB_Indication`, and `NVMCB_Store` methods to the user application. The user has to define (or rename when porting existing BLE projects) the aofrementioned functions to `USER_SVCCTL_App_Notification`, `USER_BLECB_Indication`, and `USER_NVMCB_Store` respectively. This methods allows to use BLE in the user application freely whenever BLE portion of SIdewalk is not operating. It's benefit is low efforts for integration of the existing BLE apps and full freedom of BLE usage. But it's up to the user to implement full BLE initialization and handling because this method acts like there's no BLE-related code in Sidewalk stack whenever Sidewalk-over-BLE is not used. Concurrent usage of BLE for Sidewalk and the user app is not possible for this configuration. 
- *Interleaving* - this method alllows to split BLE between Sidewalk and the user application. Concurrent usage of BLE for Sidewalk and the user app is still not possible, but Sidewalk BLE driver provides convinient API for BLE operation and handles all the necessary initializations/de-initializations and the other arrangments. The user app can use that extended API to simplify the development of the BLE-enabled applications. The main use case for this mode is when peripheral-only BLE stack variant is used and there's no support for extended advertising in it, meaning only one advertisement source (either Sidewalk or the user app) is possible at a time.
- *Concurrency* - as the name suggests, this mode supports full concurrency of Sidewalk and user app BLE operations. More over, it allows the user to run multiple BLE instances in parallel (e.g. BLE beacon, multiple peripherals, etc.). This mode requires at least Basic Plus variant of the BLE stack because extended advertising should be supported.

This sample app runs the *Concurrency* mode of BLE co-existence.

### Concept of a Virtual BLE Device

BLE co-existence defines so-called virtual BLE devices. This means Sidewalk and every instance inside the user app is treated as separate and independent BLE device. Every virtual BLE device can be initialized and terminated at any moment without impact to the other virtual devices. However, these virtual devices still run on the very same STM32WBA5x MCU and full isolation is not possible. Here are the highlights of the independent features:
- Each device may define unique advertising data and scan response data (valid for virtual Peripherals and virtual Broadcasters).
- Each device may set individual advertising parameters (valid for virtual Peripherals and virtual Broadcasters).
- Each device may define own connection parameters (valid for virtual Peripherals and virtual Centrals*).
- Each device may specify individual maximum attribute MTU size (valid for virtual Peripherals and virtual Centrals*).
- Security requirements are managed on per-device basis (e.g. one virtual device may require secure connection and pairng while the other allows direct access with no encryption on BLE level).

*Note: Central and Observer roles are not fully supported yet, but there are no technical barriers in adding them to the Sidewalk BLE driver*

This sample app can run up to four different virtual BLE devices:
- Peripheral with Sidewalk GATT profile - can be used by Sidewalk gateways only
- Broadcaster - a BLE beacon implementing an Eddystone URL format with a link to [www.st.com](https://www.st.com)
- Peripheral with maximum MTU (512 bytes) and support for secure connections - it's possible to pair with it and establish an encrypted communication channel
- Peripheral with small MTU size and no support for pairing. It allows up to two simultaneous connections to it

All the above virtual device can operate concurrently, meaning up to 4 BLE connections to the STM32WBA5x can be established (Sidewalk + Secure Peripheral + 2 x non-secure Peripheral) while still advetsing the beacon.

Here are some important limitations:
- GAP attributes are common for all virtual devices and cannot be virtualized. This means the GAP attributes will have identical value for any virtual BLE device.
- GATT database is shared between all the virtual BLE devices. This means the full list of GATT sevices can be observed via any virtual devices. However, access to GATT characteristics is controlled by the driver - even though it's possible to see the presence of the GATT service/characteristic it's still not possible to access the, For example, an attempt to access a characteristic of the Secure Peripheral via a connection to the Non-secure Peripheral will be rejected. In other words, while it is possible to spot the presence of additional characteristics, it's still not possible to access their data.
- When RPA is used for advertising all the virtual devices will share the very same RPA generated by the controller. The driver is still capable to differentiate incoming connections based on the specific advertisement, but from the Central/Observer side this will look like this is one Peripheral/Broadcaster that frequently rotates advertising data. This should not be a big deal for real-life use cases because typically advertisiements are filtered (e.g. by service UUID)
- Pairing when the virtual device uses a static random or non-resolvable private address is not working

### CLI commands usage and format
Arguments to the command are shown in <>
```
    ble init                                    - initialize the user portion of BLE
                                                    This command is independent of "sid init 1" and can be executed at any point
                                                    regardless of the state of Sidewalk

    ble deinit                                  - deinitialize the user portion of BLE
                                                    Has no impact on Sidewalk operation

    ble <virtual_device> bootstrap <device_num> - load virtual device's profile into the controller
                                                    This command allocates RAM, updates GATT database, etc, but does not
                                                    start any activities for the virtual device.

                                                    Valid devices: beacon, peripheral

                                                    <device_num> is valid for peripheral devices only because this app
                                                    supports two different peripherals

                                                    Examples:
                                                        ble beacon bootstrap
                                                        ble peripheral bootstrap 1
                                                        ble peripheral bootstrap 2

    ble <virtual_device> start <device_num>     - starts operation for the selected device
                                                    Parameters are identical to ble bootstrap command

                                                    Examples:
                                                        ble beacon start
                                                        ble peripheral start 1
                                                        ble peripheral start 2

    ble <virtual_device> stop <device_num>      - stops operation for the selected device, but does not deallocate resources

    ble <virtual_device> terminate <device_num> - fully unloads the specified device from BLE controller

    ble peripheral send <device_num> <data>     - updates a characteristic and initiates a BLE notification
                                                    This command is valid for peripheral devices only

                                                    Example:
                                                        ble peripheral send 1 hello

    ble factory_reset                           - Erase all bonds, generate new Static Random Address, and IRK and ERK keys
```

### Typical workflow
```
ble init

ble beacon bootstrap
ble periphreal bootstrap 1
ble periphreal bootstrap 2

ble beacon start
ble periphreal start 1
ble periphreal start 2

ble peripheral send 1 hello_1
ble peripheral send 2 hello_2
...
ble deinit
```