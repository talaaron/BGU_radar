# -*- coding: utf-8 -*-
"""
UART link to the MSP430.

Frame format (both directions): [HEADER][LEN][DATA...][CHK],
CHK = (LEN + sum(DATA)) & 0xFF. Every frame is answered with a raw
one-byte token: 0xCC (ACK) or 0xBB (NACK).

A background thread reads the port, answers each frame and decodes the
binary payloads. Results reach the GUI through Qt signals.
"""
import serial
import sys
import time
import threading
from typing import Optional, Tuple

from PyQt5.QtCore import QObject, pyqtSignal


SERIAL_PORT = 'COM6'
BAUD_RATE = 9600

# Frame headers
HEADER_TEXT = 0xAA  # text payload (ASCII)
HEADER_BIN = 0xFF  # binary payload (packed measurements)

# Single-byte tokens (not framed)
ACK_TOKEN_OK = 0xCC
ACK_TOKEN_FAIL = 0xBB
PC_READY_TOKEN = 0xAB  # PC -> MCU: host is ready

ACK_TIMEOUT_SEC = 2.0
MAX_RETRIES = 2  # first attempt + one retry

# If True, suppress printing normal RX frames while waiting for an ACK token.
SUPPRESS_RXF_WHILE_WAITING_ACK = False
VERBOSE_ACK_LOG = True

# The LDR calibration table arrives as five 5-byte frames (25 bytes)
BIN25_CHUNK_LEN = 5
BIN25_TOTAL_LEN = 25


class SerialSignal(QObject):
    data_received = pyqtSignal(str)
    ack_done = pyqtSignal(bool, str)  # success, message


class SerialTerminal:
    def __init__(self, port=SERIAL_PORT, baud_rate=BAUD_RATE):
        # 9600 8N1, no flow control
        self.signals = SerialSignal()
        self.ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=1,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
            inter_byte_timeout=0.2
        )
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.verbose_ack_log = True
        # Collects the 5-byte chunks of the LDR calibration table
        self._bin25_acc = bytearray()
        self._bin25_good = True

        print(f"[INFO] Connected to {port} at {baud_rate} baud.", flush=True)

        # RX thread control
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        # TX lock and ACK-token wait
        self._tx_lock = threading.Lock()
        self._ack_cond = threading.Condition()
        self._awaiting_ack = False
        self._last_ack_token: Optional[int] = None

        # RX buffer for stream parsing
        self._rx_buf = bytearray()

    def send_to_UI(self, text):
        """Send text to the GUI through the data_received signal."""
        self.signals.data_received.emit(str(text))

    # =========================
    # Utilities
    # =========================
    @staticmethod
    def sum8(bb: bytes) -> int:
        """Sum-8 modulo 256."""
        return sum(bb) & 0xFF

    @staticmethod
    def to_hex_spaced(bb: bytes) -> str:
        """Format bytes as 'aa bb cc' lowercase hex."""
        return ' '.join(f"{b:02x}" for b in bb)

    @staticmethod
    def decode_bin3(bb: bytes) -> Tuple[int, int]:
        """Decode 3-byte binary payload: [DEG(1), PERIOD(2 LE)]."""
        if len(bb) != 3:
            raise ValueError("decode_bin3 expects exactly 3 bytes")
        deg = bb[0]
        period = bb[1] | (bb[2] << 8)
        return deg, period

    @staticmethod
    def decode_bin4(bb: bytes):
        """Not used (4-byte frames are decoded by decode_bin4_light)."""
        return None

    @staticmethod
    def decode_bin5(bb: bytes):
        """Not used (5-byte frames are collected into the 25-byte table)."""
        return None

    @staticmethod
    def decode_bin4_light(bb: bytes):
        """
        Decode 4-byte packed payload produced by create_data_to_send_light():
        STR[0] = L1[7:0]
        STR[1] = L2[7:0]
        STR[2] = bits 0-1: L1[9:8], bits 2-3: L2[9:8], bits 4-7: DEG[3:0]
        STR[3] = bits 3-0: DEG[7:4] (upper nibble unused)
        Returns: (deg, l1, l2)
        """
        if len(bb) != 4:
            raise ValueError("decode_bin4_light expects exactly 4 bytes")
        l1 = bb[0] | ((bb[2] & 0x03) << 8)
        l2 = bb[1] | (((bb[2] >> 2) & 0x03) << 8)
        deg = ((bb[2] >> 4) & 0x0F) | ((bb[3] & 0x0F) << 4)  # 0..255
        return deg, l1, l2

    @staticmethod
    def decode_bin25_ldr(bb: bytes):
        """
        Decode the 25-byte LDR calibration table (pack_two_ldr_arrays_from_segc()):
        Interleaved 10-bit samples, LSB-first:
        arr1[0], arr2[0], arr1[1], arr2[1], ..., arr1[9], arr2[9]
        Returns: (arr1[10], arr2[10]) with 0..1023 values.
        """
        if len(bb) != 25:
            raise ValueError("decode_bin25_ldr expects exactly 25 bytes")

        values = []
        bitbuf = 0
        bitcount = 0
        for b in bb:
            bitbuf |= (b & 0xFF) << bitcount
            bitcount += 8
            while bitcount >= 10 and len(values) < 20:
                v = bitbuf & 0x3FF
                values.append(v)
                bitbuf >>= 10
                bitcount -= 10

        # 25 bytes hold exactly 20 values; pad just in case
        while len(values) < 20:
            values.append(0)

        arr1 = values[0::2][:10]
        arr2 = values[1::2][:10]
        return arr1, arr2

    @staticmethod
    def decode_bin6_combo(bb: bytes):
        """
        Decode 6-byte payload packed as:
        L1[9:0], L2[9:0], PERIOD[15:0], DEG[7:0]   (LSB-first, little-endian bit order)
        Returns: (deg, l1, l2, period)
        """
        if len(bb) != 6:
            raise ValueError("decode_bin6_combo expects exactly 6 bytes")

        # Read the bit stream LSB first
        bitbuf = 0
        bitcount = 0
        values = []

        def pull(nbits):
            nonlocal bitbuf, bitcount
                # Load bytes until the buffer holds nbits
            while bitcount < nbits and bytes_iter_index[0] < len(bb):
                b = bb[bytes_iter_index[0]]
                bytes_iter_index[0] += 1
                bitbuf |= (b & 0xFF) << bitcount
                bitcount += 8
            v = bitbuf & ((1 << nbits) - 1)
            bitbuf >>= nbits
            bitcount -= nbits
            return v

        bytes_iter_index = [0]
        l1 = pull(10)
        l2 = pull(10)
        period = pull(16)
        deg = pull(8)

        return deg, l1, l2, period

    # =========================
    # Lifecycle
    # =========================
    def start(self):
        self._rx_thread.start()
        time.sleep(0.05)
        self.send_pc_ready()        # the MCU does not transmit before this token
        time.sleep(0.05)

    def stop(self):
        self._stop.set()
        try:
            self._rx_thread.join(timeout=1.0)
        except:
            pass
        try:
            self.ser.close()
        except:
            pass

    def send_pc_ready(self):
        """Send PC_READY_TOKEN to the MCU."""
        with self._tx_lock:
            self.ser.write(bytes([PC_READY_TOKEN]))
            self.ser.flush()
        print("[TX ready] 0xAB", flush=True)

    # =========================
    # Low-level framing
    # =========================
    def _send_frame(self, header: int, payload: bytes):
        """Send one frame [HEADER][LEN][DATA...][CHECKSUM]."""
        if not (0 <= header <= 0xFF):
            raise ValueError("header must be 0..255")
        if len(payload) > 255:
            raise ValueError("payload length must fit in one byte")
        length = len(payload)
        chk = (length + self.sum8(payload)) & 0xFF
        frame = bytes([header, length]) + payload + bytes([chk])
        with self._tx_lock:
            self.ser.write(frame)
            self.ser.flush()

    def _send_token(self, ok: bool):
        """Send raw 1-byte token (0xCC OK or 0xBB FAIL)."""
        tok = ACK_TOKEN_OK if ok else ACK_TOKEN_FAIL
        with self._tx_lock:
            self.ser.write(bytes([tok]))
            self.ser.flush()
        if getattr(self, 'verbose_ack_log', False):
            print(f"[TX ack] {'ok' if ok else 'fail'}", flush=True)

    # =========================
    # Public TX API
    # =========================

    def send_text_frame(self, text: str):
        """
        Send textual payload (UTF-8) as framed message using HEADER_TEXT.
        Wait for a raw ACK token (0xCC/0xBB) up to ACK_TIMEOUT_SEC.
        Retry once on NACK or timeout.
        """
        if not isinstance(text, str):
            text = str(text)
        payload = text.encode('utf-8', errors='ignore')

        attempts = 0
        sent_ok = False
        cause_lines = []

        while attempts < MAX_RETRIES and not sent_ok:
            attempts += 1

                # Arm the ACK wait
            with self._ack_cond:
                self._awaiting_ack = True
                self._last_ack_token = None

            # Send the frame
            self._send_frame(HEADER_TEXT, payload)

            # Wait for the token
            ack_tok = None
            start = time.time()
            with self._ack_cond:
                remaining = ACK_TIMEOUT_SEC
                while self._last_ack_token is None and remaining > 0:
                    self._ack_cond.wait(timeout=remaining)
                    remaining = ACK_TIMEOUT_SEC - (time.time() - start)
                ack_tok = self._last_ack_token
                self._awaiting_ack = False
                self._last_ack_token = None

            if ack_tok == ACK_TOKEN_OK:
                print(f"[TX good] {text}", flush=True)
                self.signals.ack_done.emit(True, text)
                sent_ok = True
                break
            elif ack_tok == ACK_TOKEN_FAIL:
                cause_lines.append("[RX - bad] fail")
            else:
                cause_lines.append("[RX - bad] timeout")

        if not sent_ok:
            for line in cause_lines:
                print(line, flush=True)
            print(f"[TX bad] {text}", flush=True)
            self.signals.ack_done.emit(False, text)
            return False

        return True

    # =========================
    # RX parsing thread
    # =========================
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                # Block for the first byte, then take whatever else is waiting.
                # (read(64) would wait for inter_byte_timeout = 0.2 s after every
                # short frame, so each ACK to the MCU was delayed by 200 ms.)
                chunk = self.ser.read(1)
                if chunk and self.ser.in_waiting:
                    chunk += self.ser.read(self.ser.in_waiting)
                if chunk:
                    self._rx_buf.extend(chunk)
                    try:
                        self._parse_rx_buffer()
                    except Exception as e:
                        print(f"[ERR] RX parse: {e}", file=sys.stderr, flush=True)
                else:
                    time.sleep(0.005)
            except serial.SerialException as e:
                print(f"[ERR] Serial exception: {e}", file=sys.stderr, flush=True)
                time.sleep(0.2)

    def _note_token(self, tok: int):
        """Handle an incoming ACK / NACK token."""
        if tok == ACK_TOKEN_OK:
            with self._ack_cond:
                if self._awaiting_ack and self._last_ack_token is None:
                    self._last_ack_token = tok
                    self._ack_cond.notify_all()
            print("[RX - good] ack", flush=True)
        elif tok == ACK_TOKEN_FAIL:
            with self._ack_cond:
                if self._awaiting_ack and self._last_ack_token is None:
                    self._last_ack_token = tok
                    self._ack_cond.notify_all()
            print("[RX - bad] ack", flush=True)
        else:
            # Unknown byte: ignore
            pass

    def _parse_rx_buffer(self):
        """
        Extract tokens and frames:
        - Tokens: single bytes 0xCC/0xBB (not framed)
        - Frames: [HDR(0xAA/0xFF)][LEN][DATA...][CHK]
        Any other byte is discarded.
        """
        buf = self._rx_buf
        while True:
            if not buf:
                break

            # Token
            b0 = buf[0]
            if b0 in (ACK_TOKEN_OK, ACK_TOKEN_FAIL):
                del buf[0:1]
                self._note_token(b0)
                continue

            # Frame: wait until it is complete
            if b0 in (HEADER_TEXT, HEADER_BIN):
                if len(buf) < 2:
                    break  # incomplete
                hdr = buf[0]
                length = buf[1]
                total = 1 + 1 + length + 1  # HDR + LEN + DATA + CHK
                if len(buf) < total:
                    break  # incomplete
                data = bytes(buf[2:2 + length])
                chk = buf[2 + length]
                del buf[:total]
                self._handle_frame(hdr, length, data, chk)
                continue

            # Anything else: discard one byte
            del buf[0:1]


    def _handle_frame(self, header: int, length: int, data: bytes, chk: int):
        """Check one complete frame, answer it with a token, then decode and log it."""
        # Optionally hide received frames while waiting for our own ACK
        if SUPPRESS_RXF_WHILE_WAITING_ACK and getattr(self, "_awaiting_ack", False):
            # The frame must still be answered
            ok_hidden = ((length + self.sum8(data)) & 0xFF) == chk
            self._send_token(ok_hidden)
            return

        # Checksum
        calc = (length + self.sum8(data)) & 0xFF
        ok = (calc == chk)

        # Answer every frame with ACK or NACK
        self._send_token(ok)

        # Log line
        hdr_str = f"0x{header:02X}"
        len_str = f"{length}"
        data_str = self.to_hex_spaced(data)
        chk_str = f"0x{chk:02X}"
        status = "good" if ok else "bad"
        base = f"[RXF - {status}] {hdr_str} | {len_str} | {data_str} | {chk_str}"

        if not ok:
            print(base, flush=True)
            return

        if header == HEADER_TEXT:
            txt = data.decode('utf-8', errors='ignore')
            print(f"{base} | = {txt}", flush=True)
            return

        if header == HEADER_BIN:
            # 3 bytes: sonar (angle + echo width) -> distance
            if length == 3:
                try:
                    angle, period = self.decode_bin3(data)
                    time_s = period * 1e-6
                    des = (time_s * 34645) / 2.0  # distance [cm], speed of sound 346.45 m/s

                    self.send_to_UI(f"{des:.3f}")
                    print(f"{base} | = ANGLE = {angle} | PERIOD = {period} | DES = {des:.3f}", flush=True)
                except Exception as e:
                    print(f"{base} | decode error: {e}", flush=True)
                return

            # 4 bytes: light scan (angle + LDR1 + LDR2)
            if length == 4:
                try:
                    deg, l1, l2 = self.decode_bin4_light(data)

                    self.send_to_UI(f"{base} | = ANGLE = {deg} | LED1 = {l1} | LED2 = {l2}")
                    print(f"{base} | = ANGLE = {deg} | LED1 = {l1} | LED2 = {l2}", flush=True)
                except Exception:
                    print(base, flush=True)
                return

            # 6 bytes: full scan (angle + LDR1 + LDR2 + echo width)
            if length == 6:
                try:
                    deg, l1, l2, period = self.decode_bin6_combo(data)
                    time_s = period * 1e-6
                    des = (time_s * 34645) / 2.0
                    self.send_to_UI(f"{base} | = ANGLE = {deg} | LED1 = {l1} | LED2 = {l2} | PERIOD = {period} | DES = {des:.3f}")
                    print(f"{base} | = ANGLE = {deg} | LED1 = {l1} | LED2 = {l2} | PERIOD = {period} | DES = {des:.3f}",
                          flush=True)
                except Exception:
                    print(base, flush=True)
                return

            # 25 bytes: LDR calibration table (led1Arr[10], led2Arr[10])
            if length == 25 or ('BIN25_TOTAL_LEN' in globals() and length == BIN25_TOTAL_LEN):
                try:
                    arr1, arr2 = self.decode_bin25_ldr(data)
                    hex25 = self.to_hex_spaced(data)
                    arr1_str = "[" + ",".join(str(x) for x in arr1) + "]"
                    arr2_str = "[" + ",".join(str(x) for x in arr2) + "]"
                    msg = f"[RXF - good] 0xFF | 25 | {hex25} | ={{ led1Arr = {arr1_str} | led2Arr = {arr2_str} }}"
                    self.send_to_UI(msg)
                    print(msg, flush=True)
                except Exception:
                    msg = f"[RXF - good] 0xFF | 25 | {self.to_hex_spaced(data)}"
                    print(msg, flush=True)
                return

            # 5 bytes: one chunk of the calibration table; decode after 5 chunks
            if ('BIN25_CHUNK_LEN' in globals() and length == BIN25_CHUNK_LEN) or length == 5:
                if not hasattr(self, "_bin25_acc"):
                    self._bin25_acc = bytearray()
                self._bin25_acc.extend(data)

                if len(self._bin25_acc) >= 25:
                    full = bytes(self._bin25_acc[:25])
                    self._bin25_acc.clear()
                    try:
                        arr1, arr2 = self.decode_bin25_ldr(full)
                        hex25 = self.to_hex_spaced(full)
                        arr1_str = "[" + ",".join(str(x) for x in arr1) + "]"
                        arr2_str = "[" + ",".join(str(x) for x in arr2) + "]"
                        msg = f"[RXF - good] 0xFF | 25 | {hex25} | ={{ led1Arr = {arr1_str} | led2Arr = {arr2_str} }}"
                        print(msg, flush=True)
                    except Exception:
                        hex25 = self.to_hex_spaced(full)
                        msg = f"[RXF - good] 0xFF | 25 | {hex25}"
                        print(msg, flush=True)
                return

            # Unknown binary length: log raw
            print(base, flush=True)
            return

        # Unknown header: log raw
        print(base, flush=True)
