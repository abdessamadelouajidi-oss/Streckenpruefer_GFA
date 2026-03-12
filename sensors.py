"""Sensor implementations for the Raspberry Pi measurement system."""

import threading
import time
from abc import ABC, abstractmethod
from collections import deque


class Sensor(ABC):
    """Base class for sensors."""

    @abstractmethod
    def read(self):
        """Read sensor data and return it."""
        raise NotImplementedError


class Accelerometer(Sensor):
    """Interrupt-driven MMA8452Q transient detector."""

    # =========================
    # MMA8452Q registers
    # =========================
    STATUS = 0x00
    OUT_X_MSB = 0x01
    OUT_X_LSB = 0x02
    OUT_Y_MSB = 0x03
    OUT_Y_LSB = 0x04
    OUT_Z_MSB = 0x05
    OUT_Z_LSB = 0x06

    INT_SOURCE = 0x0C
    WHO_AM_I = 0x0D
    XYZ_DATA_CFG = 0x0E
    HP_FILTER_CUTOFF = 0x0F

    TRANSIENT_CFG = 0x1D
    TRANSIENT_SRC = 0x1E
    TRANSIENT_THS = 0x1F
    TRANSIENT_COUNT_REG = 0x20

    CTRL_REG1 = 0x2A
    CTRL_REG2 = 0x2B
    CTRL_REG3 = 0x2C
    CTRL_REG4 = 0x2D
    CTRL_REG5 = 0x2E

    # =========================
    # Bit fields
    # =========================
    ACTIVE_BIT = 0x01

    INT_EN_TRANS = 1 << 5
    INT_CFG_TRANS = 1 << 5

    TRANSIENT_EA = 1 << 6
    TRANSIENT_ELE = 1 << 4
    TRANSIENT_ZTEFE = 1 << 3
    TRANSIENT_YTEFE = 1 << 2
    TRANSIENT_XTEFE = 1 << 1
    TRANSIENT_HPF_BYP = 1 << 0  # keep 0 to USE HPF

    ZTRANSE = 1 << 5
    ZPOL = 1 << 4
    YTRANSE = 1 << 3
    YPOL = 1 << 2
    XTRANSE = 1 << 1
    XPOL = 1 << 0

    def __init__(
        self,
        i2c_address=0x1D,
        bus=1,
        auto_detect=True,
        int1_gpio=26,      # BCM 26 = physical pin 37
        int2_gpio=19,      # BCM 19 = physical pin 35
        use_int1=True,
        int_active_low=True,
        odr_hz=100,
        full_scale_g=8,
        threshold_mps2=10.0,
        transient_count=1,
        dead_time_s=0.05,
    ):
        self.bus_number = int(bus)
        self.i2c_address = int(i2c_address)
        self.auto_detect = bool(auto_detect)

        self.int1_gpio = int(int1_gpio)
        self.int2_gpio = int(int2_gpio)
        self.use_int1 = bool(use_int1)
        self.int_active_low = bool(int_active_low)
        self.interrupt_gpio = self.int1_gpio if self.use_int1 else self.int2_gpio

        self.odr_hz = int(odr_hz)
        self.full_scale_g = int(full_scale_g)
        self.threshold_mps2 = float(threshold_mps2)
        self.transient_count = max(1, int(transient_count))
        self.dead_time_s = float(dead_time_s)

        self.GPIO = None
        self.i2c = None

        self._bus_lock = threading.Lock()
        self._events_lock = threading.Lock()
        self._events = deque()
        self._last_event_time = 0.0
        self._last_xyz = {"x": 0.0, "y": 0.0, "z": 0.0}

        self._load_gpio()
        self._initialize_i2c()

    # =========================
    # Init helpers
    # =========================
    def _load_gpio(self):
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
        except ImportError:
            print("[ACCELEROMETER] Warning: RPi.GPIO not available, interrupts disabled")

    def _initialize_i2c(self):
        try:
            from smbus2 import SMBus

            self.i2c = SMBus(self.bus_number)
            self._detect_device_address()
            self._configure_sensor()
            self._setup_gpio_interrupt()

            print(
                "[ACCELEROMETER] Initialized on "
                f"bus {self.bus_number}, address 0x{self.i2c_address:02X}, "
                f"GPIO {self.interrupt_gpio}"
            )
        except Exception as e:
            print(f"[ACCELEROMETER] ERROR: Could not initialize - {type(e).__name__}: {e}")
            self.cleanup()

    def _detect_device_address(self):
        candidate_addresses = []

        for address in (self.i2c_address, 0x1C, 0x1D):
            if address not in candidate_addresses:
                candidate_addresses.append(address)

        if not self.auto_detect:
            candidate_addresses = [self.i2c_address]

        for address in candidate_addresses:
            try:
                who = self._read_register(self.WHO_AM_I, address=address)
                print(f"[ACCELEROMETER] WHO_AM_I at 0x{address:02X} = 0x{who:02X}")
                if who == 0x2A:
                    self.i2c_address = address
                    return
            except OSError:
                pass

        raise OSError("No MMA8452Q found at 0x1C or 0x1D")

    # =========================
    # Low-level I2C
    # =========================
    def _read_register(self, reg, address=None):
        if self.i2c is None:
            raise OSError("I2C bus not available")
        target = self.i2c_address if address is None else address
        with self._bus_lock:
            return self.i2c.read_byte_data(target, reg)

    def _write_register(self, reg, value):
        if self.i2c is None:
            raise OSError("I2C bus not available")
        with self._bus_lock:
            self.i2c.write_byte_data(self.i2c_address, reg, value & 0xFF)

    def _read_block(self, reg, length):
        if self.i2c is None:
            raise OSError("I2C bus not available")
        with self._bus_lock:
            return self.i2c.read_i2c_block_data(self.i2c_address, reg, length)

    # =========================
    # Conversions
    # =========================
    @staticmethod
    def _odr_to_ctrl_reg1_bits(odr_hz):
        mapping = {
            800: 0b000,
            400: 0b001,
            200: 0b010,
            100: 0b011,
            50:  0b100,
            12:  0b101,
            13:  0b101,
            6:   0b110,
            2:   0b111,
            1:   0b111,
        }
        if odr_hz not in mapping:
            raise ValueError("Unsupported ODR_HZ")
        return mapping[odr_hz] << 3

    @staticmethod
    def _range_to_xyz_data_cfg(full_scale_g):
        mapping = {
            2: 0x00,
            4: 0x01,
            8: 0x02,
        }
        if full_scale_g not in mapping:
            raise ValueError("FULL_SCALE_G must be 2, 4, or 8")
        return mapping[full_scale_g]

    @staticmethod
    def _threshold_mps2_to_ths_lsb(threshold_mps2):
        g_value = threshold_mps2 / 9.80665
        threshold = round(g_value / 0.063)
        return max(1, min(127, threshold))

    @staticmethod
    def _twos_complement_12bit(value):
        if value & 0x800:
            value -= 0x1000
        return value

    # =========================
    # Sensor config
    # =========================
    def _set_standby(self):
        ctrl1 = self._read_register(self.CTRL_REG1)
        self._write_register(self.CTRL_REG1, ctrl1 & ~self.ACTIVE_BIT)
        time.sleep(0.01)

    def _set_active(self):
        ctrl1 = self._read_register(self.CTRL_REG1)
        self._write_register(self.CTRL_REG1, ctrl1 | self.ACTIVE_BIT)
        time.sleep(0.01)

    def _configure_sensor(self):
        who = self._read_register(self.WHO_AM_I)
        if who != 0x2A:
            raise RuntimeError(f"Unexpected WHO_AM_I: 0x{who:02X}")

        threshold_lsb = self._threshold_mps2_to_ths_lsb(self.threshold_mps2)
        threshold_g = threshold_lsb * 0.063
        threshold_mps2_actual = threshold_g * 9.80665

        print("[ACCELEROMETER] Config:")
        print(f"  Range            : +/-{self.full_scale_g} g")
        print(f"  ODR              : {self.odr_hz} Hz")
        print(f"  Threshold req    : {self.threshold_mps2:.2f} m/s^2")
        print(f"  Threshold reg    : {threshold_lsb}")
        print(f"  Threshold actual : {threshold_mps2_actual:.2f} m/s^2")
        print(f"  Count            : {self.transient_count}")
        print(f"  Route            : {'INT1' if self.use_int1 else 'INT2'}")
        print(f"  GPIO             : {self.interrupt_gpio}")

        self._set_standby()

        # Set measurement range
        self._write_register(
            self.XYZ_DATA_CFG,
            self._range_to_xyz_data_cfg(self.full_scale_g),
        )

        # Set ODR, remain in standby while configuring
        ctrl1_value = self._odr_to_ctrl_reg1_bits(self.odr_hz)
        self._write_register(self.CTRL_REG1, ctrl1_value)

        # Enable transient detection on X/Y/Z, latch events, use HPF
        self._write_register(
            self.TRANSIENT_CFG,
            self.TRANSIENT_ELE
            | self.TRANSIENT_ZTEFE
            | self.TRANSIENT_YTEFE
            | self.TRANSIENT_XTEFE
        )

        # DBCNTM=1 + threshold bits
        self._write_register(self.TRANSIENT_THS, 0x80 | threshold_lsb)

        # Debounce count
        self._write_register(self.TRANSIENT_COUNT_REG, self.transient_count)

        # Enable transient interrupt
        ctrl4 = self._read_register(self.CTRL_REG4)
        self._write_register(self.CTRL_REG4, ctrl4 | self.INT_EN_TRANS)

        # Route interrupt to INT1 or INT2
        ctrl5 = self._read_register(self.CTRL_REG5)
        if self.use_int1:
            ctrl5 |= self.INT_CFG_TRANS
        else:
            ctrl5 &= ~self.INT_CFG_TRANS
        self._write_register(self.CTRL_REG5, ctrl5)

        # Activate sensor
        self._set_active()

        # Give filter/startup a moment
        time.sleep(0.2)

        # Clear any stale startup flags
        try:
            self._read_register(self.INT_SOURCE)
        except Exception:
            pass

        startup_src = self._read_register(self.TRANSIENT_SRC)
        print(f"[ACCELEROMETER] Startup TRANSIENT_SRC clear read = 0x{startup_src:02X}")

    # =========================
    # GPIO interrupt setup
    # =========================
    def _setup_gpio_interrupt(self):
        if self.GPIO is None:
            return

        self.GPIO.setwarnings(False)
        self.GPIO.setmode(self.GPIO.BCM)

        try:
            self.GPIO.cleanup(self.interrupt_gpio)
        except Exception:
            pass

        pull = self.GPIO.PUD_UP if self.int_active_low else self.GPIO.PUD_DOWN
        self.GPIO.setup(self.interrupt_gpio, self.GPIO.IN, pull_up_down=pull)

        edge = self.GPIO.FALLING if self.int_active_low else self.GPIO.RISING

        self.GPIO.add_event_detect(
            self.interrupt_gpio,
            edge,
            callback=self._gpio_callback,
            bouncetime=1,
        )

    def _gpio_callback(self, channel):
        del channel

        if self.i2c is None:
            return

        try:
            self._handle_transient_event()
        except Exception as e:
            print(f"[ACCELEROMETER] Error in GPIO callback: {e}")

    # =========================
    # Reading acceleration
    # =========================
    def _read_raw_xyz_counts(self):
        data = self._read_block(self.OUT_X_MSB, 6)

        x_value = ((data[0] << 8) | data[1]) >> 4
        y_value = ((data[2] << 8) | data[3]) >> 4
        z_value = ((data[4] << 8) | data[5]) >> 4

        x_value = self._twos_complement_12bit(x_value)
        y_value = self._twos_complement_12bit(y_value)
        z_value = self._twos_complement_12bit(z_value)

        return x_value, y_value, z_value

    def _counts_to_mps2(self, counts):
        counts_per_g = {
            2: 1024.0,
            4: 512.0,
            8: 256.0,
        }[self.full_scale_g]
        return (counts / counts_per_g) * 9.80665

    def _read_xyz_mps2(self):
        x_counts, y_counts, z_counts = self._read_raw_xyz_counts()
        return (
            self._counts_to_mps2(x_counts),
            self._counts_to_mps2(y_counts),
            self._counts_to_mps2(z_counts),
        )

    # =========================
    # Event handling
    # =========================
    def _handle_transient_event(self):
        now = time.monotonic()

        if (now - self._last_event_time) < self.dead_time_s:
            return

        # Read and clear interrupt/transient source
        try:
            self._read_register(self.INT_SOURCE)
        except Exception:
            pass

        src = self._read_register(self.TRANSIENT_SRC)

        if not (src & self.TRANSIENT_EA):
            return

        x_value, y_value, z_value = self._read_xyz_mps2()

        reading = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "x": round(x_value, 2),
            "y": round(y_value, 2),
            "z": round(z_value, 2),
        }

        with self._events_lock:
            self._events.append(reading)

        self._last_xyz = dict(reading)
        self._last_event_time = now

        print(
            f"[ACCELEROMETER] TRANSIENT "
            f"x={reading['x']:+.2f} "
            f"y={reading['y']:+.2f} "
            f"z={reading['z']:+.2f}"
        )

    def read(self):
        """Read current acceleration directly."""
        if self.i2c is None:
            return {"x": 999.0, "y": 999.0, "z": 999.0}

        try:
            x_value, y_value, z_value = self._read_xyz_mps2()
            self._last_xyz = {
                "x": round(x_value, 2),
                "y": round(y_value, 2),
                "z": round(z_value, 2),
            }
            return dict(self._last_xyz)
        except Exception as e:
            print(f"[ACCELEROMETER] Read error: {e}")
            return {"x": 999.0, "y": 999.0, "z": 999.0}

    def drain_events(self):
        """Return and clear all queued transient-triggered readings."""
        with self._events_lock:
            events = list(self._events)
            self._events.clear()
        return events

    def clear_events(self):
        with self._events_lock:
            self._events.clear()

    def cleanup(self):
        if self.GPIO is not None:
            try:
                self.GPIO.remove_event_detect(self.interrupt_gpio)
            except Exception:
                pass
            try:
                self.GPIO.cleanup(self.interrupt_gpio)
            except Exception:
                pass

        if self.i2c is not None:
            try:
                self.i2c.close()
            except Exception:
                pass
            self.i2c = None


class HallSensor:
    """
    Threaded polling hall sensor.
    Counts ONLY ONE per interaction:
      - counts on HIGH -> LOW
      - then locks until signal returns to HIGH and stays stable
    """

    def __init__(
        self,
        pin,
        pull_up=True,
        name="HALL_SENSOR",
        poll_hz=800,
        stable_samples=5,
    ):
        self.pin = int(pin)
        self.pull_up = bool(pull_up)
        self.name = name
        self.poll_hz = int(poll_hz)
        self.stable_samples = max(1, int(stable_samples))

        self.GPIO = None
        self._count = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        try:
            import RPi.GPIO as GPIO

            self.GPIO = GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            try:
                GPIO.cleanup(self.pin)
            except Exception:
                pass

            pull = GPIO.PUD_UP if self.pull_up else GPIO.PUD_DOWN
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull)

            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

            print(
                f"[{self.name}] Polling GPIO {self.pin} at ~{self.poll_hz} Hz "
                f"(stable_samples={self.stable_samples})"
            )
        except ImportError:
            print(f"[{self.name}] Warning: RPi.GPIO not available, using simulated mode")
        except Exception as e:
            print(f"[{self.name}] Warning: Could not initialize - {type(e).__name__}: {e}")

    def _run(self):
        period = 1.0 / self.poll_hz if self.poll_hz > 0 else 0.001
        armed = True
        high_streak = 0
        last = self.GPIO.input(self.pin)

        while not self._stop.is_set():
            cur = self.GPIO.input(self.pin)

            if armed:
                if last == 1 and cur == 0:
                    with self._lock:
                        self._count += 1
                    armed = False
                    high_streak = 0
            else:
                if cur == 1:
                    high_streak += 1
                    if high_streak >= self.stable_samples:
                        armed = True
                        high_streak = 0
                else:
                    high_streak = 0

            last = cur
            time.sleep(period)

    def get_count(self):
        with self._lock:
            return self._count

    def reset_count(self):
        with self._lock:
            self._count = 0

    def cleanup(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        if self.GPIO is None:
            return

        try:
            self.GPIO.cleanup(self.pin)
        except Exception:
            pass