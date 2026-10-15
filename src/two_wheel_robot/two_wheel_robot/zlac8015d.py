"""ZLAC8015D dual-channel hub motor driver over Modbus RTU."""

import math

import minimalmodbus


class ZLAC8015DController:
    def __init__(self, port: str, device_id: int = 1, baudrate: int = 115200,
                 wheel_diameter_m: float = 0.1, gear_ratio: float = 1.0, max_rpm: float = 100.0):
        """
        Args:
            port: Serial port device path (e.g. '/dev/ttyUSB1')
            device_id: Modbus device ID (0 = broadcast, no readback)
            baudrate: Serial communication speed
            wheel_diameter_m: Wheel diameter in meters
            gear_ratio: Motor RPM / Wheel RPM (1.0 = direct drive)
            max_rpm: Maximum motor RPM limit
        """
        self.port = port
        self.device_id = device_id
        self.baudrate = baudrate
        self.wheel_diameter_m = wheel_diameter_m
        self.gear_ratio = gear_ratio
        self.max_rpm = max_rpm
        self.instr = self._init_instrument(device_id)

    def _init_instrument(self, modbus_id):
        instr = minimalmodbus.Instrument(self.port, modbus_id)
        instr.serial.baudrate = self.baudrate
        instr.serial.bytesize = 8
        instr.serial.parity = minimalmodbus.serial.PARITY_NONE
        instr.serial.stopbits = 1
        instr.serial.timeout = 0.05  # sufficient for 115200 baud
        instr.mode = minimalmodbus.MODE_RTU
        instr.close_port_after_each_call = False  # keep port open for faster transactions
        return instr

    def close(self):
        if self.instr.serial is not None:
            self.instr.serial.close()

    # ======== ID / readback ========
    def read_device_id(self):
        return self.instr.read_register(0x2001)

    def read_velocity_command(self):
        """Read back the left and right target RPM registers."""
        values = self.instr.read_registers(0x2088, 2)
        return tuple(self._to_signed_16(value) for value in values)

    @staticmethod
    def _to_signed_16(value):
        return value - 0x10000 if value & 0x8000 else value

    # ======== Control modes ========
    def set_velocity_mode(self):
        # Batch write: mode=3 (velocity), enable=8
        self.instr.write_registers(0x200D, [3, 8])

    def set_position_mode(self):
        self.instr.write_registers(0x200D, [1, 8])

    def set_torque_mode(self):
        self.instr.write_registers(0x200D, [4, 8])

    # ======== Physical unit conversion ========
    def mps_to_rpm(self, speed_mps: float) -> int:
        """RPM = (v / wheel_circumference) * 60 * gear_ratio, clamped to +-max_rpm."""
        if self.wheel_diameter_m <= 1e-9:
            return 0
        rpm = (speed_mps / (math.pi * self.wheel_diameter_m)) * 60.0 * self.gear_ratio
        return int(max(-self.max_rpm, min(self.max_rpm, rpm)))

    # ======== Motor commands ========
    def set_velocity_mps(self, left_mps: float, right_mps: float):
        left_rpm = self.mps_to_rpm(left_mps)
        right_rpm = self.mps_to_rpm(right_mps)
        self.set_velocity(left_rpm, right_rpm)
        return left_rpm, right_rpm

    def set_velocity(self, left_rpm: int, right_rpm: int):
        # Both velocities in one transaction
        self._write_registers_signed(0x2088, [int(left_rpm), int(right_rpm)])

    def set_position(self, left_pulses: int, right_pulses: int, speed_rpm: int = 100):
        left_val = left_pulses & 0xFFFFFFFF
        right_val = right_pulses & 0xFFFFFFFF
        self.instr.write_registers(0x208A, [
            (left_val >> 16) & 0xFFFF, left_val & 0xFFFF,
            (right_val >> 16) & 0xFFFF, right_val & 0xFFFF,
            speed_rpm, speed_rpm,
        ])
        self.instr.write_register(0x200E, 0x11)  # start left
        self.instr.write_register(0x200E, 0x12)  # start right

    def set_torque(self, left_ma: int, right_ma: int):
        self._write_registers_signed(0x2090, [left_ma, right_ma])

    def stop_motor(self):
        self.instr.write_register(0x200E, 7)

    def _write_registers_signed(self, start_register, values):
        self.instr.write_registers(start_register, [v & 0xFFFF for v in values])
