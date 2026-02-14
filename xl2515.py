"""
xl2515.py  (MicroPython)

XL2515 (MCP2515-compatible) CAN controller driver for Pico-class boards.

Highlights:
- Standard (11-bit) frame support
- Configurable bitrate from known-good tables
- RX filtering for standard IDs (RXB0 only)
- Error detection helpers

Background
----------
The RP2040/RP2350 family does NOT have a built-in CAN controller.
This driver talks to an external XL2515/MCP2515 over SPI and a transceiver.

Default pin mapping (SPI1)
--------------------------
These defaults match a common RP2350-CAN wiring:

- SCK  -> GPIO10
- MOSI -> GPIO11
- MISO -> GPIO12
- CS   -> GPIO9
- INT  -> GPIO8   (interrupt from CAN controller when RX buffer gets a frame)

If your board is wired differently, override pins in the constructor.
"""

from machine import Pin, SPI
import time

__version__ = "0.2.0"


class XL2515CAN:
    """
    Minimal XL2515/MCP2515 CAN driver.

    Supported features (good for many hobby projects):
    - Standard 11-bit CAN identifiers (0x000 .. 0x7FF)
    - Sending data frames (0..8 bytes)
    - Receiving frames from RX buffer 0 (optional, interrupt-assisted)
    - Basic mask/filter (RXF0 + RXM0) for standard IDs
    - Error detection and reporting

    Not implemented (can be added later):
    - Extended 29-bit identifiers
    - Remote frames (RTR)
    - RX buffer 1 usage, advanced filtering
    """

    # ===== MCP2515 / XL2515 SPI commands =====
    _CMD_RESET  = 0xC0
    _CMD_READ   = 0x03
    _CMD_WRITE  = 0x02
    _CMD_BITMOD = 0x05

    # ===== Register addresses =====
    _REG_CANSTAT = 0x0E
    _REG_CANCTRL = 0x0F

    _REG_CNF3 = 0x28
    _REG_CNF2 = 0x29
    _REG_CNF1 = 0x2A

    _REG_CANINTE = 0x2B
    _REG_CANINTF = 0x2C
    _REG_EFLG    = 0x2D

    # TX buffer 0
    _REG_TXB0CTRL = 0x30
    _REG_TXB0SIDH = 0x31
    _REG_TXB0SIDL = 0x32
    _REG_TXB0EID8 = 0x33
    _REG_TXB0EID0 = 0x34
    _REG_TXB0DLC  = 0x35
    _REG_TXB0D0   = 0x36  # start of data bytes D0..D7

    # RX buffer 0
    _REG_RXB0CTRL = 0x60
    _REG_RXB0SIDH = 0x61
    _REG_RXB0SIDL = 0x62
    _REG_RXB0DLC  = 0x65
    _REG_RXB0D0   = 0x66

    # Filters / masks
    _REG_RXF0SIDH = 0x00
    _REG_RXF0SIDL = 0x01
    _REG_RXM0SIDH = 0x20
    _REG_RXM0SIDL = 0x21

    # ===== Bit definitions =====
    _TXREQ = 0x08          # TXBnCTRL: transmit request
    _RX0IF = 0x01          # CANINTF: RX buffer 0 full interrupt flag
    _RX0IE = 0x01          # CANINTE: RX buffer 0 full interrupt enable
    _MERRF = 0x80          # CANINTF: message error flag
    _ERRIF = 0x20          # CANINTF: error interrupt flag

    # EFLG register bits
    _EFLG_RX0OVR = 0x40    # RX buffer 0 overflow
    _EFLG_RX1OVR = 0x80    # RX buffer 1 overflow
    _EFLG_TXBO   = 0x20    # Bus-off
    _EFLG_TXEP   = 0x10    # TX error passive
    _EFLG_RXEP   = 0x08    # RX error passive

    # CANCTRL operation mode bits (REQOP[7:5])
    _REQOP_MASK   = 0xE0
    _MODE_NORMAL  = 0x00
    _MODE_CONFIG  = 0x80
    _MODE_LISTEN  = 0x60

    # CLKOUT enable bit
    _CLKOUT_ENABLED = 0x04  # Bit 2 in CANCTRL: enables clock output on CLKOUT pin

    # RXB0CTRL configuration values
    _RXB0CTRL_ACCEPT_ALL = 0x60    # Bits[6:5]=11: receive all messages (filters disabled)
    _RXB0CTRL_USE_FILTERS = 0x00   # Bits[6:5]=00: receive only filtered frames
    
    # RXB0SIDL standard frame marker
    _RXB0SIDL_STANDARD = 0x60      # EXIDE=0 (bit 3), SRR=1 (bit 4), IDE=1 (bit 5)

    # ===== Bitrate table (from common MicroPython examples) =====
    # Values are [CNF1, CNF2, CNF3]
    # IMPORTANT: This table is "board + oscillator" dependent.
    BITRATES = {
        "5KBPS"   : (0xBF, 0xFF, 0x87),
        "10KBPS"  : (0x5F, 0xFF, 0x87),
        "20KBPS"  : (0x18, 0xA4, 0x04),
        "50KBPS"  : (0x09, 0xA4, 0x04),
        "100KBPS" : (0x04, 0x9E, 0x03),
        "125KBPS" : (0x03, 0x9E, 0x03),
        "250KBPS" : (0x01, 0x1E, 0x03),
        "500KBPS" : (0x00, 0x9E, 0x03),
        "800KBPS" : (0x00, 0x92, 0x02),
        "1000KBPS": (0x00, 0x82, 0x02),
    }

    def __init__(
        self,
        rate_kbps: str = "500KBPS",
        spi_port: int = 1,
        spi_freq: int = 10_000_000,
        pin_sck: int = 10,
        pin_mosi: int = 11,
        pin_miso: int = 12,
        pin_cs: int = 9,
        pin_int = 8,
        rx_filter_id = None,
        rx_mask_11bit: int = 0x7FF,
        accept_all: bool = True,
        tx_timeout_ms: int = 50,
        clkout_enabled: bool = True,
    ):
        """
        Create driver and initialize the CAN controller.

        Parameters
        ----------
        rate_kbps:
            String key from BITRATES, e.g. "500KBPS".
        spi_port, spi_freq:
            SPI peripheral and frequency. 10 MHz is typically safe.
        pin_*:
            GPIO pin numbers.
        pin_int:
            If you pass None, interrupt setup is skipped and you can poll manually.
        rx_filter_id:
            If not None, sets RXF0 to match this standard CAN ID.
            If None and accept_all=True, RX accepts everything.
        rx_mask_11bit:
            11-bit mask (default 0x7FF = all bits must match for the filter to hit)
        accept_all:
            True  -> configure RXB0CTRL to receive all messages
            False -> configure RXB0CTRL to use filters
        tx_timeout_ms:
            Max time to wait for a free TX buffer before raising TimeoutError.
        clkout_enabled:
            Enable the controller's CLKOUT pin if your board uses it.
        """
        if tx_timeout_ms <= 0:
            raise ValueError("tx_timeout_ms must be > 0")

        self._tx_timeout_ms = tx_timeout_ms
        self._clkout_enabled = clkout_enabled

        # --- SPI bus setup ---
        self.spi = SPI(
            spi_port,
            baudrate=spi_freq,
            polarity=0,
            phase=0,
            bits=8,
            sck=Pin(pin_sck),
            mosi=Pin(pin_mosi),
            miso=Pin(pin_miso),
        )

        # Chip Select pin (active low)
        self.cs = Pin(pin_cs, Pin.OUT, value=1)

        # --- Interrupt pin setup (optional) ---
        self._rx_flag = False
        self.int = None
        if pin_int is not None:
            self.int = Pin(pin_int, Pin.IN, Pin.PULL_UP)
            # IRQ handler MUST be tiny: just set a flag.
            self.int.irq(handler=self._int_callback, trigger=Pin.IRQ_FALLING)

        # --- Hardware init of the controller ---
        self.reset()
        time.sleep_ms(50)

        self.set_bitrate(rate_kbps)

        # Setup RX behavior
        self.configure_rx(
            filter_id=rx_filter_id,
            mask_11bit=rx_mask_11bit,
            accept_all=accept_all,
        )

        # Enable RX interrupt flag inside the controller
        self.write_reg(self._REG_CANINTF, 0x00)  # clear flags
        self.write_reg(self._REG_CANINTE, self._RX0IE)

        # Go to normal mode so it can actually transmit on the CAN bus.
        self.set_mode_normal(clkout_enabled=clkout_enabled)

    # -----------------------
    # Low-level SPI functions
    # -----------------------
    def _select(self) -> None:
        self.cs.value(0)

    def _deselect(self) -> None:
        self.cs.value(1)

    def reset(self) -> None:
        """Reset the CAN controller via SPI command."""
        self._select()
        self.spi.write(bytearray([self._CMD_RESET]))
        self._deselect()

    def read_reg(self, reg: int) -> int:
        """Read a single register byte."""
        self._select()
        self.spi.write(bytearray([self._CMD_READ, reg & 0xFF]))
        val = self.spi.read(1)[0]
        self._deselect()
        return val

    def write_reg(self, reg: int, val: int) -> None:
        """Write a single register byte."""
        self._select()
        self.spi.write(bytearray([self._CMD_WRITE, reg & 0xFF, val & 0xFF]))
        self._deselect()

    def _write_regs_sequential(self, start_reg: int, data) -> None:
        """
        Write multiple consecutive registers in one SPI transaction.
        
        This is much faster than calling write_reg() multiple times.
        Example: writing 8 data bytes takes 1 transaction instead of 8.
        """
        self._select()
        self.spi.write(bytearray([self._CMD_WRITE, start_reg & 0xFF]))
        self.spi.write(data)
        self._deselect()

    def _read_regs_sequential(self, start_reg: int, length: int) -> bytes:
        """Read multiple consecutive registers in one SPI transaction."""
        self._select()
        self.spi.write(bytearray([self._CMD_READ, start_reg & 0xFF]))
        data = self.spi.read(length)
        self._deselect()
        return data

    def bit_modify(self, reg: int, mask: int, val: int) -> None:
        """
        Modify individual bits of a register (built-in MCP2515 command).
        Only bits set in mask are updated.
        """
        self._select()
        self.spi.write(bytearray([self._CMD_BITMOD, reg & 0xFF, mask & 0xFF, val & 0xFF]))
        self._deselect()

    # -----------------------
    # Controller configuration
    # -----------------------
    def set_bitrate(self, rate_kbps: str) -> None:
        """
        Program the CAN bit timing registers (CNF1/2/3).
        Typically you call this once at startup.

        The values come from BITRATES and are known-good for common examples.
        """
        if rate_kbps not in self.BITRATES:
            raise ValueError("Unknown bitrate: %s" % rate_kbps)

        cnf1, cnf2, cnf3 = self.BITRATES[rate_kbps]
        self.write_reg(self._REG_CNF1, cnf1)
        self.write_reg(self._REG_CNF2, cnf2)
        self.write_reg(self._REG_CNF3, cnf3)

    def set_mode_normal(self, clkout_enabled: bool = True) -> None:
        """
        Put the controller in NORMAL mode.
        In NORMAL mode it can transmit/receive on the bus.

        If your design doesn't need CLKOUT, set clkout_enabled=False.
        
        Raises RuntimeError if mode change fails after 3 attempts.
        """
        canctrl = self._MODE_NORMAL
        if clkout_enabled:
            canctrl |= self._CLKOUT_ENABLED

        # Try up to 3 times to enter normal mode
        for attempt in range(3):
            self.write_reg(self._REG_CANCTRL, canctrl)
            time.sleep_ms(10)
            
            mode = self.read_reg(self._REG_CANSTAT) & self._REQOP_MASK
            if mode == self._MODE_NORMAL:
                return
        
        raise RuntimeError("Failed to enter NORMAL mode after 3 attempts")

    def set_mode_config(self, clkout_enabled: bool = True) -> None:
        """Put the controller in CONFIG mode (no TX/RX)."""
        canctrl = self._MODE_CONFIG
        if clkout_enabled:
            canctrl |= self._CLKOUT_ENABLED

        for _ in range(3):
            self.write_reg(self._REG_CANCTRL, canctrl)
            time.sleep_ms(10)
            mode = self.read_reg(self._REG_CANSTAT) & self._REQOP_MASK
            if mode == self._MODE_CONFIG:
                return
        raise RuntimeError("Failed to enter CONFIG mode after 3 attempts")

    def configure_rx(self, filter_id = None, mask_11bit: int = 0x7FF, accept_all: bool = True) -> None:
        """
        Configure RX buffer 0 filtering.

        - accept_all=True  -> RXB0CTRL = 0x60 (accept everything, no filtering)
        - accept_all=False -> RXB0CTRL = 0x00 (use RXF0/RXM0 filters)

        If filter_id is provided, RXF0 is set to that standard ID and RXM0 to mask_11bit.
        """
        # Configure whether to use filters or accept all
        self.write_reg(self._REG_RXB0CTRL, 
                      self._RXB0CTRL_ACCEPT_ALL if accept_all else self._RXB0CTRL_USE_FILTERS)

        # If user wants a filter, program RXF0 + RXM0 for a standard ID.
        if filter_id is not None:
            if not (0 <= filter_id <= 0x7FF):
                raise ValueError("filter_id must be 0..0x7FF")

            if not (0 <= mask_11bit <= 0x7FF):
                raise ValueError("mask_11bit must be 0..0x7FF")

            # Convert 11-bit ID into (SIDH, SIDL) format used by MCP2515 registers.
            f_sidh = (filter_id >> 3) & 0xFF
            f_sidl = (filter_id & 0x07) << 5
            m_sidh = (mask_11bit >> 3) & 0xFF
            m_sidl = (mask_11bit & 0x07) << 5

            self.write_reg(self._REG_RXF0SIDH, f_sidh)
            self.write_reg(self._REG_RXF0SIDL, f_sidl)
            self.write_reg(self._REG_RXM0SIDH, m_sidh)
            self.write_reg(self._REG_RXM0SIDL, m_sidl)

    # -----------------------
    # Transmit / Receive
    # -----------------------
    @staticmethod
    def _encode_std_id(can_id: int) -> tuple[int, int]:
        """Standard 11-bit CAN ID to (SIDH, SIDL)."""
        if not (0 <= can_id <= 0x7FF):
            raise ValueError("CAN ID must be 0..0x7FF (standard)")
        sidh = (can_id >> 3) & 0xFF
        sidl = (can_id & 0x07) << 5
        return sidh, sidl

    def send(self, can_id: int, data) -> None:
        """
        Send one standard CAN data frame.

        can_id: 11-bit standard identifier (0..0x7FF)
        data: iterable of bytes (list, bytes, bytearray). Max length 8.
        
        Raises:
            ValueError: if can_id or data is invalid
            TimeoutError: if TX buffer is busy for > 50ms
        """
        # Convert data to a bytearray (MicroPython-friendly, mutable, indexable)
        if isinstance(data, bytearray):
            payload = data
        else:
            payload = bytearray(data)

        if len(payload) > 8:
            raise ValueError("CAN payload must be 0..8 bytes")

        # Wait until TX buffer 0 is free (TXREQ bit clears when not pending)
        start = time.ticks_ms()
        while self.read_reg(self._REG_TXB0CTRL) & self._TXREQ:
            if time.ticks_diff(time.ticks_ms(), start) >= self._tx_timeout_ms:
                raise TimeoutError("TX buffer busy - frame not sent")
            time.sleep_ms(1)

        # Program ID
        sidh, sidl = self._encode_std_id(can_id)
        self.write_reg(self._REG_TXB0SIDH, sidh)
        self.write_reg(self._REG_TXB0SIDL, sidl)

        # Extended ID registers unused for standard frames
        self.write_reg(self._REG_TXB0EID8, 0x00)
        self.write_reg(self._REG_TXB0EID0, 0x00)

        # DLC = number of bytes
        self.write_reg(self._REG_TXB0DLC, len(payload) & 0x0F)

        # IMPROVED: Write all data bytes in one transaction
        if len(payload) > 0:
            self._write_regs_sequential(self._REG_TXB0D0, payload)

        # Request transmission by setting TXREQ without clobbering other bits
        self.bit_modify(self._REG_TXB0CTRL, self._TXREQ, self._TXREQ)

    def recv(self):
        """
        Non-blocking receive from RX buffer 0.
        
        Design notes:
        - If INT pin is configured: _rx_flag provides a fast gate to avoid SPI reads
          when no data is available. We still confirm with CANINTF to handle race
          conditions (interrupt fires, we read flag=True, but CANINTF shows no data yet).
        - If no INT pin: pure polling mode, checks CANINTF every call.
        
        This approach gets best of both worlds: interrupt efficiency + polling reliability.

        Returns:
            None                        -> nothing received
            (can_id: int, data: bytes)  -> one received frame
        """
        # Always verify with hardware register.
        # Note: relying only on the software flag can miss pending frames.
        if (self.read_reg(self._REG_CANINTF) & self._RX0IF) == 0:
            # False alarm from interrupt, or no data in polling mode
            if self.int is not None:
                self._rx_flag = False
            return None
        
        # Confirmed: data is available. Clear the software flag.
        if self.int is not None:
            self._rx_flag = False

        # Read header (SIDH, SIDL, EID8, EID0, DLC) in one transaction
        header = self._read_regs_sequential(self._REG_RXB0SIDH, 5)
        sid_h = header[0]
        sid_l = header[1]
        can_id = (sid_h << 3) | (sid_l >> 5)
        dlc = header[4] & 0x0F

        if dlc:
            data = self._read_regs_sequential(self._REG_RXB0D0, dlc)
        else:
            data = b""

        # Clear RX0IF (and other interrupt flags if any)
        self.bit_modify(self._REG_CANINTF, self._RX0IF, 0x00)

        return can_id, bytes(data)

    # -----------------------
    # Error detection (NEW)
    # -----------------------
    def check_errors(self) -> dict:
        """
        Check for CAN controller error conditions.
        
        Returns:
            Dictionary with error flags:
            - rx0_overflow: RX buffer 0 overrun (messages lost)
            - rx1_overflow: RX buffer 1 overrun
            - tx_error: Transmit error occurred
            - rx_passive: In RX error passive state
            - tx_passive: In TX error passive state
            - bus_off: Bus-off condition (controller disabled)
        """
        canintf = self.read_reg(self._REG_CANINTF)
        eflg = self.read_reg(self._REG_EFLG)
        
        return {
            'rx0_overflow': bool(eflg & self._EFLG_RX0OVR),
            'rx1_overflow': bool(eflg & self._EFLG_RX1OVR),
            'tx_error': bool(canintf & (self._ERRIF | self._MERRF)),
            'rx_passive': bool(eflg & self._EFLG_RXEP),
            'tx_passive': bool(eflg & self._EFLG_TXEP),
            'bus_off': bool(eflg & self._EFLG_TXBO)
        }
    
    def clear_errors(self) -> None:
        """
        Clear error flags in EFLG and CANINTF registers.
        
        IMPORTANT: Only clears error-related bits in CANINTF, NOT RX/TX flags.
        This prevents accidentally discarding a pending received frame.
        """
        # Clear all error flags in EFLG
        self.write_reg(self._REG_EFLG, 0x00)
        
        # Only clear error-related bits in CANINTF, preserving RX/TX flags
        # Clear: ERRIF (0x20), MERRF (0x80)
        # Preserve: RX0IF (0x01), RX1IF (0x02), TX0IF (0x04), TX1IF (0x08), TX2IF (0x10)
        error_bits = self._ERRIF | self._MERRF
        self.bit_modify(self._REG_CANINTF, error_bits, 0x00)

    # -----------------------
    # Cleanup (NEW)
    # -----------------------
    def close(self) -> None:
        """
        Shut down the CAN controller gracefully.
        - Disables interrupts
        - Puts controller in CONFIG mode (stops TX/RX)
        """
        self.write_reg(self._REG_CANINTE, 0x00)  # Disable all interrupts
        self.set_mode_config(clkout_enabled=self._clkout_enabled)

    # -----------------------
    # Interrupt callback
    # -----------------------
    def _int_callback(self, pin):
        """
        Called by MicroPython ISR when INT pin falls.
        MUST be tiny: set a flag and exit.
        """
        self._rx_flag = True


def hexdump(data) -> str:
    """Helper: convert bytes into a space-separated hex string."""
    return " ".join("%02X" % b for b in data)
