MicroPython CAN Utilities (Pico/RP2040/RP2350)
==============================================

This repo contains a small CAN controller driver for XL2515 / MCP2515-compatible
controllers over SPI.

Highlights
----------
- Standard 11-bit CAN frames (TX/RX)
- Known-good bitrate table for common MCP2515 setups
- RX filtering (RXB0 only)
- Error flag helpers and safe shutdown

Hardware
--------
Board: RP2040/RP2350 + external XL2515 + transceiver

Default pins (SPI1):
- SCK  -> GPIO10
- MOSI -> GPIO11
- MISO -> GPIO12
- CS   -> GPIO9
- INT  -> GPIO8

If your board is wired differently, pass custom pins to the driver constructor.

Quick Start
-----------
Copy these files to the device filesystem:
- xl2515.py

Example (basic TX):
    from xl2515 import XL2515CAN

    can = XL2515CAN(rate_kbps="500KBPS", accept_all=True)
    can.send(0x123, bytes([0x01, 0x02, 0x03]))
    can.close()

Bitrate Notes
-------------
The BITRATES table is board + oscillator dependent. It is tuned to common
MCP2515 examples and assumes a typical oscillator.
If you use a different board or oscillator, update the table.

Limitations
-----------
- Standard 11-bit identifiers only
- RX buffer 0 only (RXB1 is not used)
- No remote frames or extended IDs

License
-------
Add your preferred license before publishing publicly.
# xl2515-micropython
