################################################################################
# Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
# (now owned by Analog Devices Inc.),
#
# Copyright © 2023 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################

"""
Move a motor back and forth on a Raspberry Pi 4 using
- pytrinamic for UART register access (TMC2300)
- GPIO STEP/DIR/EN for motion control

Pins:
- GPIO24: ENABLE
- GPIO18: STEP
- GPIO23: DIR
"""
import struct
import time
from serial import Serial
import RPi.GPIO as GPIO
import pytrinamic
from pytrinamic.ic import TMC2300


UART_DEVICE = "/dev/serial0"
UART_BAUDRATE = 250000
UART_TIMEOUT_S = 0.2
UART_DRIVER_ADDRESS = 0  # 0-3 based on AD0/AD1 straps

PIN_EN = 24
PIN_STEP = 18
PIN_DIR = 23

# Motion model inputs
FSTEP_PER_ROT = 200  # full steps per revolution
CURRENT_USTEP = 16   # microsteps per full step
ROT_PER_SEC = 5.0    # revolutions per second
MOVE_ANGLE_DEG = 360.0  # angle to move in degrees (positive or negative)
ENABLE_INTERPOLATION = True  # enable microstep interpolation (INTPOL)


def _compute_crc8_atm(data: bytes, initial_value: int = 0) -> int:
    crc = initial_value
    for byte in data:
        for _ in range(8):
            if (crc >> 7) ^ (byte & 0x01):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            byte >>= 1
    return crc


class Tmc2300UartDirect:
    """Minimal UART-IC adapter to use pytrinamic TMC2300 directly."""

    def __init__(self, port: str, baudrate: int = 250000, address: int = 0, timeout_s: float = 0.2):
        self._address = address & 0xFF
        self._ser = Serial(port, baudrate, timeout=timeout_s)

    def close(self):
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _uart_write(self, data: bytes) -> None:
        self._ser.write(data)

    def _uart_read(self, length: int) -> bytes:
        return self._ser.read(length)

    def _read_reply(self) -> bytes:
        deadline = time.time() + (self._ser.timeout or 0.2)
        buf = bytearray()
        while time.time() < deadline and len(buf) < 12:
            chunk = self._uart_read(12 - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.001)
        raw = bytes(buf)
        if len(raw) >= 12:
            return raw[4:12]
        if len(raw) >= 8:
            return raw[-8:]
        return raw

    def write_drv(self, register_address: int, value: int, module_id: int = 1):
        _ = module_id
        value &= 0xFFFFFFFF
        frame = bytearray(8)
        frame[0] = 0x05
        frame[1] = self._address
        frame[2] = (register_address | 0x80) & 0xFF
        frame[3:7] = value.to_bytes(4, "big", signed=False)
        frame[7] = _compute_crc8_atm(frame[:-1])
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._uart_write(frame)
        time.sleep(0.002)

    def read_drv(self, register_address: int, module_id: int = 1, signed: bool = False):
        _ = module_id
        frame = bytearray(4)
        frame[0] = 0x05
        frame[1] = self._address
        frame[2] = register_address & 0xFF
        frame[3] = _compute_crc8_atm(frame[:-1])
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._uart_write(frame)
        time.sleep(0.002)
        reply = self._read_reply()
        if len(reply) < 8:
            raise RuntimeError(f"UART reply too short: {reply}")
        if reply[-1] != _compute_crc8_atm(reply[:-1]):
            raise RuntimeError("UART CRC mismatch")
        data = reply[3:7]
        if signed:
            return struct.unpack(">i", data)[0]
        return int.from_bytes(data, "big", signed=False)


def set_field(value: int, mask: int, shift: int, field_value: int) -> int:
    value &= ~mask
    value |= (field_value << shift) & mask
    return value


def read_register(interface: Tmc2300UartDirect, reg: int) -> int:
    return interface.read_drv(reg)


def write_register(interface: Tmc2300UartDirect, reg: int, val: int) -> None:
    interface.write_drv(reg, val)


def configure_microsteps(interface: Tmc2300UartDirect, microsteps: int = 256) -> None:
    # MRES encoding for TMC2300 (same as TMC2209):
    # 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=1
    mres_map = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
    mres = mres_map.get(microsteps, 0)

    reg_addr, mask, shift = TMC2300.FIELD.MRES
    chopconf = read_register(interface, reg_addr)
    chopconf = set_field(chopconf, mask, shift, mres)
    write_register(interface, reg_addr, chopconf)


def configure_interpolation(interface: Tmc2300UartDirect, enabled: bool) -> None:
    reg_addr, mask, shift = TMC2300.FIELD.INTPOL
    chopconf = read_register(interface, reg_addr)
    chopconf = set_field(chopconf, mask, shift, 1 if enabled else 0)
    write_register(interface, reg_addr, chopconf)


class StepperController:
    def __init__(self, pin_en: int, pin_step: int, pin_dir: int):
        self._pin_en = pin_en
        self._pin_step = pin_step
        self._pin_dir = pin_dir

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._pin_en, GPIO.OUT)
        GPIO.setup(self._pin_step, GPIO.OUT)
        GPIO.setup(self._pin_dir, GPIO.OUT)

    def enable(self, enable: bool) -> None:
        GPIO.output(self._pin_en, GPIO.HIGH if enable else GPIO.LOW)

    def _step_pulses(self, steps: int, step_hz: float) -> None:
        if steps == 0:
            return
        if step_hz <= 0:
            raise ValueError("step_hz must be > 0")
        delay = 1.0 / (step_hz * 2.0)
        for _ in range(abs(steps)):
            GPIO.output(self._pin_step, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self._pin_step, GPIO.LOW)
            time.sleep(delay)

    def move_rotations(
        self,
        move_rot: float,
        rot_per_sec: float,
        fstep_per_rot: int,
        current_ustep: int,
    ) -> None:
        if fstep_per_rot <= 0 or current_ustep <= 0:
            raise ValueError("fstep_per_rot and current_ustep must be > 0")
        if rot_per_sec <= 0:
            raise ValueError("rot_per_sec must be > 0")

        steps_per_rot = fstep_per_rot * current_ustep
        total_steps = int(round(move_rot * steps_per_rot))
        step_hz = rot_per_sec * steps_per_rot

        GPIO.output(self._pin_dir, GPIO.HIGH if total_steps >= 0 else GPIO.LOW)
        self._step_pulses(total_steps, step_hz)

    def cleanup(self) -> None:
        GPIO.cleanup()


pytrinamic.show_info()

stepper = StepperController(PIN_EN, PIN_STEP, PIN_DIR)

with Tmc2300UartDirect(UART_DEVICE, UART_BAUDRATE, UART_DRIVER_ADDRESS, UART_TIMEOUT_S) as my_interface:
    print("UART direct ready")

    # Configure driver via UART
    configure_microsteps(my_interface, CURRENT_USTEP)
    configure_interpolation(my_interface, ENABLE_INTERPOLATION)

    # Enable motor output
    stepper.enable(True)

    # Rotate forward for a while
    print("Rotating...")
    move_rot = MOVE_ANGLE_DEG / 360.0
    stepper.move_rotations(move_rot, ROT_PER_SEC, FSTEP_PER_ROT, CURRENT_USTEP)

    print("Stopping...")
    time.sleep(1.0)

    print("Moving back to 0...")
    stepper.move_rotations(-move_rot, ROT_PER_SEC, FSTEP_PER_ROT, CURRENT_USTEP)

    print("Reached position 0")

    stepper.enable(False)

stepper.cleanup()

print("\nReady.")
