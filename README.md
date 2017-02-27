Linux kernel module implementation for ADF7021 High Performance Narrow-Band Transceiver IC.
This module will create a character device to represent a single ADF7021 transceiver and
communicate with it by using kernel's GPIO pins.


Pin configuration
=================

(for Raspberry Pi 1 rev. B)

`
[AV Connector]

+--------+--------+
|        |        |
|        |   GND  |
|   26   |   25   |
+--------+--------+
|        |        |
|        |        |
|   24   |   23   |
+--------+--------+
| GPIO 25|        |
|TXRX_SW |        |
|   22   |   21   |
+--------+--------+
|        |        |
|  GND   |        |
|   20   |   19   |
+--------+--------+
| GPIO 24|        |
|   CE   |  3V3   |
|   18   |   17   |
+--------+--------+
| GPIO 23| GPIO 22|
|  SCLK  | SREAD  |
|   16   |   15   |
+--------+--------+
|        |*GPIO 21|
|  GND   | SDATA  |
|   14   |   13   |
+--------+--------+
| GPIO 18| GPIO 17|
|  SLE   |TXRX DTA|
|   12   |   11   |
+--------+--------+
|        |        |
|        |  GND   |
|   10   |    9   |
+--------+--------+
|        | GPIO 4 |
|        |TXRX CLK|
|    8   |    7   |
+--------+--------+
|        |        |
|   GND  |        |
|    6   |    5   |
+--------+--------+
|        |        |
|   5V   |        |
|    4   |    3   |
+--------+--------+
|        |        |
|   5V   |   3V3  |
|    2   |    1   |
+--------+--------+

GPIO 21 is only available on pin 13 in RPI1

`

References
==========

Datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/ADF7021.pdf