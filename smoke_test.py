"""
smoke_test.py

Minimal CAN smoke test for the XL2515/MCP2515 driver.
Sends a single frame, waits briefly, then shuts down.
"""

import time
from xl2515 import XL2515CAN, hexdump


def main():
    can = XL2515CAN(rate_kbps="500KBPS", accept_all=True)
    try:
        payload = bytes([0x01, 0x02, 0x03, 0x04])
        can.send(0x123, payload)
        print("TX 0x123 [%d] %s" % (len(payload), hexdump(payload)))
        time.sleep_ms(100)
    finally:
        try:
            can.close()
            print("CAN controller stopped")
        except:
            pass


if __name__ == "__main__":
    main()
