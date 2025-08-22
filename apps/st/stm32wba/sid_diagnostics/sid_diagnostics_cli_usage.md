# Sidewalk Diagnostics CLI Documentation

## Root Commands
 - `print`        - [Print Child Commands](#print-child-commands)
 - `tm`           - [Tm Child Commands](#tm-child-commands)
 - `gpio`         - [Gpio Child Commands](#gpio-child-commands)


## Print Child Commands
```
print - Print values from device
    fwversion   - Print FW version - print fwversion
```

## Tm Child Commands
```
 tm - RF test commands
    phy                  - Get/Set physical layer radio settings
                            modem mode - tm phy modem <mode> ; only support 0:FSK 1:LORA"
                            Freq Power - tm phy fp <mode> <freq> <power>
                            Modulation params - tm phy mod <mode> {[FSK: <br> <fdev> <mod_shaping> <bw>] [LORA: <sf> <cr> <bw>]}
                            Packet params - tm phy pkt <mode> {[FSK: <preamble_len> <preamble_min_detect> <sync_word_len> <addr_comp>
                                                                     <header_type> <payload_len> <crc_type> <radio_whitening_mode>]
                                                               [LORA: <preamble_length> <header_type> <crc_mode> <invert_iq> <payload_len>]}
                            Modulation profile params - tm phy profile <mode> <profile>
                            Lora Cad params - tm phy cad <symbol_num> <peak_detect> <min_detect> <exit_mode> <timeout>
                            Modem tx timeout - tm phy txtmout <mode> <timeout in seconds>
                            TOA for packet length - tm phy tmonair <mode> <len>
    cw                   - Turn on/off continuous carrier wave - tm cw <cwmode> ; 0: disable 1:enable
    mod                  - Turn on/off continuous modulated tx - tm mod <en> [packet_spacing in ms] [rssi threshold] [sense_duration us] [pkt counts]
    state                - Get/Set Radio state - tm state <val>
    ping                 - Send PING packet for roundtrip RSSI test - tm ping <ptmode> [<num of itr> <initial delay> <period>]
    pwr                  - Set test mode tx power - tm pwr <pwr>
    scan                 - Do a channel scan and dump the noise floor values - tm scan <start channel> <end channel> ; min channel 0 max channel 109
    fhop                 - Turn on/off frequency hopping test - tm fhop <en> [<start freq> <end freq> <separation> <dwell time>]
    reset_rx_counters    - Reset RX counters - tm reset_rx_counters
    print_rx_counters    - Print RX counters since last reset - tm print_rx_counters
    last_packet          - Print status of last tx or rx packet - tm last_packet
    last_rssi            - Print RSSI of last rx packets - tm last_rssi [rssi opt]; 0: all, 1: last 10 records
```

## Gpio Child Commands
```
gpio - GPIO test commands
    pin_test    - Set/Get gpio pin test - gpio pin_test <gpio_num> [<0, 1>]
