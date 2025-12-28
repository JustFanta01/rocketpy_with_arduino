from __future__ import annotations

import struct
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

# ----['m' = mask]----
m_FRAME_TYPE = 1 << 7  # b0=1 [x... ....]
FRAME_TYPE_DATA = 0
FRAME_TYPE_CMD  = 1
def set_frame_type(header, val):
    if val == FRAME_TYPE_DATA:
        header &= (m_FRAME_TYPE ^ 0xFF)
    if val == FRAME_TYPE_CMD:
        header |= m_FRAME_TYPE
    return header
def get_frame_type(header):
    t = (header & m_FRAME_TYPE) >> 7
    return t

# DATA
m_DATA_SUBTYPE      = 1 << 6
DATA_SUBTYPE_REQ    = 0
DATA_SUBTYPE_RESP   = 1
def set_data_subtype(header, val):
    if val == DATA_SUBTYPE_REQ:
        header &= (m_DATA_SUBTYPE ^ 0xFF)
    if val == DATA_SUBTYPE_RESP:
        header |= m_DATA_SUBTYPE
    return header
def get_data_subtype(header):
    t = (header & m_DATA_SUBTYPE) >> 6
    return t


def set_data_payload_len(header, data_len):
    if not (0 <= data_len <= 63):
        raise ValueError("data_len must be 0..63")  
    header |= data_len
    return header
def get_data_payload_len(header):
    t = header & 0x3F
    return t

# CMD ids
@dataclass
class CommandHandlers:
    on_open_main: Optional[Callable[[], None]] = None
    on_open_drogue: Optional[Callable[[], None]] = None
    on_set_air_brakes: Optional[Callable[[float], None]] = None

m_CMD_TYPE          = 0x70 # 0b0111_0000
CMD_OPEN_MAIN       = 0b000
CMD_OPEN_DROGUE     = 0b001
CMD_SET_AIR_BRAKES  = 0b010
CMD_PRINT           = 0b011
def set_cmd(header, cmd):
    if not (0 <= cmd <= 7):
        raise ValueError("cmd_id must be 0..7")
    header |= cmd << 4
    return header
def get_cmd(header):
    t = (header & 0x70) >> 4
    return t

def set_cmd_payload_len(header, payload_len):
    if not (0 <= payload_len <= 15):
        raise ValueError("payload_len must be 0..15")
    header |= payload_len
    return header
def get_cmd_payload_len(header):
    t = header & 0x0F
    return t


# ----['s' = struct]----
# Simulation payload: ax ay az pressure lat lon alt -> 7 float32 (28 bytes)
s_SIMULATION_PAYLOAD = struct.Struct("<7f")
s_SIMULATION_LEN = s_SIMULATION_PAYLOAD.size  # 28 bytes

# Airbrake payload: deployment level, float32 (4 bytes), 0..1
s_AIRBRAKE_PAYLOAD = struct.Struct("<f")  # 4 bytes
s_AIRBRAKE_LEN = s_AIRBRAKE_PAYLOAD.size

def build_data_header(is_response: bool, data_len: int) -> int:
    header = 0x00
    header = set_frame_type(header, FRAME_TYPE_DATA)
    header = set_data_subtype(header, DATA_SUBTYPE_RESP if is_response else DATA_SUBTYPE_REQ)
    header = set_data_payload_len(header, data_len)
    return header
def build_cmd_header(cmd_id: int, payload_len: int) -> int:
    header = 0x00
    header = set_frame_type(header, FRAME_TYPE_CMD)
    header = set_cmd(header, cmd_id)
    header = set_cmd_payload_len(header, payload_len)
    return header


class OneSlotMailbox:
    """
    Single-slot mailbox storing exactly one payload bytes object.

    Producer (simulation/controller):
      - put(payload): blocking, waits empty
      - try_put(payload): non-blocking

    Consumer (serial thread):
      - try_take(): non-blocking
    """
    def __init__(self):
        self._cv = threading.Condition()
        self._full = False
        self._value: Optional[bytes] = None

    def put(self, value: bytes) -> None:
        with self._cv:
            while self._full:
                self._cv.wait()
            # print("[py] communication.mailbox.put")
            self._value = value
            self._full = True
            self._cv.notify_all()

    def try_put(self, value: bytes) -> bool:
        with self._cv:
            if self._full:
                # print("[py] communication.mailbox.try_put: False")
                return False
            self._value = value
            self._full = True
            self._cv.notify_all()
            # print("[py] communication.mailbox.try_put: True")
            return True

    def try_take(self) -> Optional[bytes]:
        with self._cv:
            if not self._full:
                # print("[py] communication.mailbox.try_take: False")
                return None
            v = self._value
            self._value = None
            self._full = False
            self._cv.notify_all()
            # print("[py] communication.mailbox.try_take: True")
            return v


def start_serial_io_thread(
    mailbox: OneSlotMailbox,
    handlers: Optional[CommandHandlers] = None,
    port: str = PORT,
    baud: int = BAUD,
):
    """
    Serial IO thread with byte-collector FSM.

    - On DATA REQ (ESP32->Python): respond immediately with DATA RESP,
      with data_len=0 if no payload available, else data_len=len(payload).
    - On CMD: execute immediately; if payload exists, read it based on payload_len.
    """
    handlers = handlers or CommandHandlers()
    ser = serial.Serial(port, baud, timeout=1)

    # FSM state
    WAIT_HEADER = 0
    WAIT_PAYLOAD = 1
    
    state = WAIT_HEADER
    cur_is_cmd = False
    cur_cmd_id = 0
    cur_data_is_resp = False
    payload_len = 0
    buf = bytearray()

    def dispatch_cmd(cmd_id: int, payload: bytes):
        if cmd_id == CMD_OPEN_MAIN:
            if handlers.on_open_main:
                handlers.on_open_main()

        elif cmd_id == CMD_OPEN_DROGUE:
            if handlers.on_open_drogue:
                handlers.on_open_drogue()

        elif cmd_id == CMD_SET_AIR_BRAKES:
            if len(payload) == 4 and handlers.on_set_air_brakes:
                (level,) = s_AIRBRAKE_PAYLOAD.unpack(payload)
                handlers.on_set_air_brakes(float(level))

        elif cmd_id == CMD_PRINT:
            # print(f"[ESP32] {payload}")
            # print(f"[ESP32] {repr(payload)}")
            print(f'[ESP32] {payload.decode("UTF-8")}')
            # print(f"len(payload): {len(payload)}")
            # print(f"[ESP32] {payload!r}")

        else: 
            # print("[py]: command {:03b} not recognized.".format(cmd_id))
            print(f"[py]: command='{cmd_id}' with payload='{payload}' not recognized.")

    def run():
        # TODO: stop the thread when simulation finished!

        nonlocal state, cur_is_cmd, cur_cmd_id, cur_data_is_resp, payload_len, buf
        while True:
            b = ser.read(1)
            if not b:
                continue
            byte = b[0]

            if state == WAIT_HEADER:
                header = byte
                t = get_frame_type(header)
                if t == FRAME_TYPE_CMD:
                    cur_is_cmd = True
                    cur_cmd_id = get_cmd(header)
                    payload_len = get_cmd_payload_len(header)
                    buf = bytearray()
                    if payload_len == 0:
                        dispatch_cmd(cur_cmd_id, b"")
                        state = WAIT_HEADER
                    else:
                        state = WAIT_PAYLOAD
                elif t == FRAME_TYPE_DATA:
                    cur_is_cmd = False
                    assert get_data_subtype(header) == DATA_SUBTYPE_REQ, f"[py] arrived a DATA RESP from ESP32! h={header:08b}"

                    # check if simulation data are available
                    payload = mailbox.try_take()
                    if payload is None:
                        h = build_data_header(is_response=True, data_len=0)
                        # print(f"[py] rcv REQ. Sent 'NO': {h:08b}")
                        # print("\u2718", end = '', flush=True)
                        ser.write(bytes([h]))
                    else:
                        assert len(payload) == 28, "7 floats = 28 Bytes"
                        h = build_data_header(is_response=True, data_len=len(payload))
                        # print(f"[py] rcv REQ. Sent 'OK'={h:08b} + 'data'={payload}")
                        # print("\u2714", end = '', flush=True)
                        ser.write(bytes([h]) + payload)

                    state = WAIT_HEADER
            elif state == WAIT_PAYLOAD:
                buf.append(byte)
                if len(buf) >= payload_len:
                    payload = bytes(buf[:payload_len])
                    if cur_is_cmd:
                        dispatch_cmd(cur_cmd_id, payload)
                    else:
                        raise ValueError("dispatching data payload")
                    # else:
                    #     dispatch_data(cur_data_is_resp, payload)
                    state = WAIT_HEADER
            else:
                raise ValueError("[py]: wrong FSM state '{state}'")

    th = threading.Thread(target=run, name="SerialIO", daemon=True)
    th.start()
    return th, ser
