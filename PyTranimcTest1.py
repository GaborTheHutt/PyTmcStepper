################################################################################
# Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
# (now owned by Analog Devices Inc.),
#
# Copyright © 2023 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################

"""
Dump all register values of the TMC2300 IC via Raspberry Pi UART using pytrinamic.

Wiring (single-wire UART):
- TMC2300 PDN_UART -> Pi TXD (GPIO14) through ~1k resistor
- TMC2300 PDN_UART -> Pi RXD (GPIO15) direct
- Common GND between Pi and driver
"""
import struct
import time
from serial import Serial
import pytrinamic
from pytrinamic.evalboards import TMC2300_eval


UART_DEVICE = "/dev/serial0"
UART_BAUDRATE = 250000
UART_TIMEOUT_S = 0.2
UART_DRIVER_ADDRESS = 0  # 0-3 based on AD0/AD1 straps


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
	"""Minimal UART-IC adapter to use pytrinamic TMC2300_eval without TMCL."""

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
		"""Read reply; handle echo (4 bytes) if present."""
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

	def __str__(self):
		return f"Connection: type=uart_direct port={self._ser.port} baudrate={self._ser.baudrate}"


pytrinamic.show_info()

with Tmc2300UartDirect(UART_DEVICE, UART_BAUDRATE, UART_DRIVER_ADDRESS, UART_TIMEOUT_S) as my_interface:
	print(my_interface)
	
	eval_board = TMC2300_eval(my_interface)
	drv = eval_board.ics[0]
	print("Driver info: " + str(drv.get_info()))
	print("Register dump for " + str(drv.get_name()) + ":")

	print("GCONF:       0x{0:08X}".format(eval_board.read_register(drv.REG.GCONF)))
	print("GSTAT:       0x{0:08X}".format(eval_board.read_register(drv.REG.GSTAT)))
	print("IFCNT:       0x{0:08X}".format(eval_board.read_register(drv.REG.IFCNT)))
	print("SLAVECONF:   0x{0:08X}".format(eval_board.read_register(drv.REG.SLAVECONF)))
	print("IHOLD_IRUN:  0x{0:08X}".format(eval_board.read_register(drv.REG.IHOLD_IRUN)))
	print("TPOWERDOWN:  0x{0:08X}".format(eval_board.read_register(drv.REG.TPOWERDOWN)))
	print("TSTEP:       0x{0:08X}".format(eval_board.read_register(drv.REG.TSTEP)))
	print("TCOOLTHRS:   0x{0:08X}".format(eval_board.read_register(drv.REG.TCOOLTHRS)))
	print("MSCNT:       0x{0:08X}".format(eval_board.read_register(drv.REG.MSCNT)))
	print("MSCURACT:    0x{0:08X}".format(eval_board.read_register(drv.REG.MSCURACT)))
	print("CHOPCONF:    0x{0:08X}".format(eval_board.read_register(drv.REG.CHOPCONF)))
	print("DRV_STATUS:  0x{0:08X}".format(eval_board.read_register(drv.REG.DRV_STATUS)))
	print("PWM_CONF:    0x{0:08X}".format(eval_board.read_register(drv.REG.PWM_CONF)))
	print("PWM_SCALE:   0x{0:08X}".format(eval_board.read_register(drv.REG.PWM_SCALE)))
	print("PWM_AUTO:    0x{0:08X}".format(eval_board.read_register(drv.REG.PWM_AUTO)))

print("\nReady.")
