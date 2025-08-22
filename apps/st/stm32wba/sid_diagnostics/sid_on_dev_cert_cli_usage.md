# Sidewalk On-Device Certificate Generation CLI Documentation

For additional information see - https://wiki.labcollab.net/confluence/display/HALO/On-Device+Certificate+Generation#OnDeviceCertificateGeneration-CLICommandsDescription

## Root Command
 - `cert`        - [Cert Child Commands](#cert-child-commands)

## Cert Child Commands
```
cert    - On-Device Certificate Generation commands
    init    - Initialization of On-Device Certificate Generation library
                Format: cert init
    deinit  - Deinitialization of On-Device Certificate Generation Library
                Format: cert deinit
    smsn    - Generate Sidewalk Manufacturing Serial Number (SMSN)
                Format: cert smsn <device_type> <dsn> <apid> [<board_id>]
                Parameters:
                    <device_type> - Device type issued by DMS when a new device is on-boarded.
                    <dsn> - A customer provided Device Serial Number (DSN).
                    <apid> - Advertised Product ID.
                    <board_id> - [optional] This field can be used to indicate the type of build of the device, HVT, EVT, DVT, PVT or MP.
    csr     - Generate Certificate Signing Request (CSR)
                Format: cert csr <curve>
                Parameters:
                        <curve> Crypto algorithm for which the user wants to get the request. Can be p256r1 or ed25519.
    chain   - Write Sidewalk Certificate Chain
        start   - Start writing the chain
                    Format: cert chain start <curve>
                    Parameters:
                        <curve> Crypto algorithm for which the user wants to get the request. Can be p256r1 or ed25519.
        write   - Write data to chain
                    Format: cert chain write <base64string>
                    Parameters:
                        <base64string> Fragment of base64 string to be written to the chain.
        commit  - Commit pre-written data to chain - cert chain commit
    app_key - Write application server ED25519 public key
        start   - Start writing ED25519 public key
                    Format: cert app_key start
        write   - Write data to app public key
                    Format: cert app_key write <base64string>
                    Parameters:
                        <base64string> Fragment of base64 string to be written to the key
        commit  - Commit pre-written data to key
                    Format: cert app_key commit
    store   - Verify and store Sidewalk certificates
                Format: cert store
```
## General conditions

1. All responses related to On-Device Certificate Generation commands are output in the format:
```
{CERT ...}
```
This lets us safely multiplex the certificate command output with regular log messages and CLI echoes etc on the same RTT console.

2. Each correct command as a result of execution receives in response the result of the command completion (successful or not):
```
{CERT OK}
{CERT ERROR N}
```
where N is an error code (see enum sid_error_t)

3. If the command needs to output additional data, then they will be printed before the output of the command completion tag. For example:
```
{CERT some data1}
{CERT some data2}
...
{CERT OK}
```
4. If the output of the command expects data in base64 format, this data will be enclosed between the characters <...>. For example:
```
{CERT <base64string>}
```
5. Because of CLI limits, the base64 data has to be sent as multiple fragments. As a result of executing the command,
the output base64 fragments must be concatenated, for example:
```
{CERT <base64string1>}
{CERT <base64string2>}
{CERT <base64string3>}
{CERT OK}
```
Command output result: Resultbase64string = base64string1 + base64string2 + base64string3

The max size of base64 fragment is determined by the 'BASE64_MTU' parameter returned by cert init command. If not specified, then the user must use
a maximum size of fragments of no more than 32 characters.

## Usage scenario and examples

1. Initialize the On-Device Certificate Generation library
```
--> cert init
[00034914] <info> app: {CERT BASE64_MTU=32}
[00034914] <info> app: {CERT OK}
```
2. Generate a Sidewalk Manufacturing Serial Number
```
--> cert smsn "A232AX65BNIW2J" "G6F1JN06119201GP" "zGhh"
[00021770] <info> app: {CERT <r+2+ioRPp5oYqmEDCVNZLIe1AzkxQ1HA>}
[00021770] <info> app: {CERT <xJjvLqv6zN0=>}
[00021770] <info> app: {CERT OK}
```
Where output SMSN is:
"r+2+ioRPp5oYqmEDCVNZLIe1AzkxQ1HAxJjvLqv6zN0=" (base64 string format) or after convertion to hex string format:
"AFEDBE8A844FA79A18AA61030953592C87B50339314351C0C498EF2EABFACCDD"

**Note:** When generating SMSN, the application adds a suffix "-PRODUCTION" to <device_type>. As a result, the SMSN generation formula looks like:
```
SMSN = SHA256(<device_type>-PRODUCTION || <dsn> || <apid> || [<board_id>])
```
3. Generate a Certifiate Signing Request for each supported curves (ed25519 or p256r1)
```
--> cert csr "ed25519"
[01003024] <info> app: {CERT <r+2+ioRPp5oYqmEDCVNZLIe1AzkxQ1HA>}
[01003024] <info> app: {CERT <xJjvLqv6zN02QhVhanDd+59/lut9PPEd>}
[01003024] <info> app: {CERT <iNsyzd2Q+Ob0OgjU2/Nl6DFi8EB+Af7U>}
[01003025] <info> app: {CERT <vpUbH/UPEubyft9c2Rokh/KXTayPA8xx>}
[01003025] <info> app: {CERT <0cRL+M08vnm8QP3Xt8Pc0nyj0JnNSBzi>}
[01003025] <info> app: {CERT <FzairsLOMQ4=>}
[01003025] <info> app: {CERT OK}

--> cert csr "p256r1"
[00959472] <info> app: {CERT <r+2+ioRPp5oYqmEDCVNZLIe1AzkxQ1HA>}
[00959472] <info> app: {CERT <xJjvLqv6zN3FgDCM3EjEoRWxrNXIX1jS>}
[00959472] <info> app: {CERT <HZ7IAyCEJgdh22LzT29f5GFDHYy3tYwW>}
[00959472] <info> app: {CERT <lBi3kMm5pzWGKOGXBSZGtmnhoX8qr56E>}
[00959473] <info> app: {CERT <Pj35kFZ8NPN5S1bO9n13V8Fb1qZKt8ls>}
[00959473] <info> app: {CERT <1O61fXHhmfMh0TyFKBSqErz11B7uuYkX>}
[00959473] <info> app: {CERT <CZGzyWE6VRIc2MUhCbTWjQ==>}
[00959473] <info> app: {CERT OK}
```
4. Pass the generated CSRs to the Sidewalk Signing Tool and get back the signed Sidewalk Certificate Chains from the Tool.

5. Write the received signed certificate chains to the device. Do this for both ed25519 and p256r1 crypto algorithms.

Start writing ed25519 chain:
```
--> cert chain start "ed25519"
[01360358] <info> app: {CERT OK}
```

Write all fragments of the chain:
```
--> cert chain write "r+2+ioRPp5oYqmEDCVNZLIe1AzkxQ1HA"
[01364657] <info> app: {CERT OK}
--> cert chain write "xJjvLqv6zN02QhVhanDd+59/lut9PPEd"
[01367941] <info> app: {CERT OK}
--> cert chain write "iNsyzd2Q+Ob0OgjU2/Nl6Iub3NJzi2sR"
[01370619] <info> app: {CERT OK}
--> cert chain write "DTGXABkeUa7LqlfeskZ8CSVufW6mbj7d"
[01375030] <info> app: {CERT OK}
...
...
--> cert chain write "oXDl2XPaVD4CvvLearrOSlFv+lsNbC4r"
[01382891] <info> app: {CERT OK}
--> cert chain write "gZn23MtIBM/7YQmJwmQ+FXRup6Tkubg1"
[01382892] <info> app: {CERT OK}
--> cert chain write "hpz04J/09dxg8UiZmntHiUr1GfkTOFMY"
[01382892] <info> app: {CERT OK}
--> cert chain write "qRB+Aw=="
[01382893] <info> app: {CERT OK}
```

Commit the chain:
```
--> cert chain commit
[01390329] <info> app: {CERT OK}
```
Repeat these steps for the p256r1 curve.

**Note:** If the user runs `cert chain commit` command, the library checks the previously written data (length, matching of the key and SMSN, etc.).
If the data is incorrect, then the commit will fail. In this case, all buffers will be cleared and the user must re-initiate the writing
by the `cert chain start <curve>` command.
Also, the commit will fail if the user does not generate a CSR before.

6. Write application server ED25519 public key to the device

```
--> cert app_key start
[01062055] <info> app: {CERT OK}
--> cert app_key write "NTCo2qTi8Pc2WUyGBvzPJlnX+yjIX24f"
[01064239] <info> app: {CERT OK}
--> cert app_key write "9mmoYkTf/Lc="
[01066284] <info> app: {CERT OK}
--> cert app_key commit
[01068309] <info> app: {CERT OK}
```
7. Check the certificates and save all the data in the MFG storage
```
--> cert store
[00304038] <info> app: {CERT OK}
```
**Note:** Depending on the platform, this command may take a long time to execute (up to several seconds).

8. Deinitialize the library to free resources
```
--> cert deinit
[00124589] <info> app: {CERT OK}
```
