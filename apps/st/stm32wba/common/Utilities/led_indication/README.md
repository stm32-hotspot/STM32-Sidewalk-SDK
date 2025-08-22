# LED indication

Used only Green and Red LEDs, because Blue LED line intersects with SPI.

## LED States and Actions

The table below describes the statuses and actions of the LEDs

NUMBER | STATE | ACTION
--- | --- | ---
1 | LED_INDICATE_IDLE | Green LED blinks once every 2 sec as a hearbeat to see that the board is alive
2 | LED_INDICATE_BONDING | Green LED blinks quickly until it connects
3 | LED_INDICATE_CONNECTED | Green LED turn on
4 | LED_INDICATE_SENT_OK | Green LED flashes super quickly 3 times
5 | LED_INDICATE_SEND_ERROR | Red LED flash super quickly 3 times
6 | LED_INDICATE_RCV_OK | Green LED flashes super quickly 3 times
7 | LED_INDICATE_RCV_ERROR | Red LED flash super quickly 3 times
8 | LED_INDICATE_ERROR | Red LED turn on
--- 
