"""Main application loop for the Raspberry Pi measurement system."""

import csv
import os
import shutil
import time

from buttons import BeginButton, PowerButton
from config import (
    ACCELEROMETER_AUTO_DETECT,
    ACCELEROMETER_DEAD_TIME_S,
    ACCELEROMETER_FULL_SCALE_G,
    ACCELEROMETER_I2C_ADDRESS,
    ACCELEROMETER_I2C_BUS,
    ACCELEROMETER_INT1_GPIO,
    ACCELEROMETER_INT2_GPIO,
    ACCELEROMETER_INT_ACTIVE_LOW,
    ACCELEROMETER_ODR_HZ,
    ACCELEROMETER_THRESHOLD_MPS2,
    ACCELEROMETER_TRANSIENT_COUNT,
    ACCELEROMETER_USE_INT1,
    BEGIN_BUTTON_PIN,
    CSV_OUTPUT_PATH,
    HALL_ENABLED,
    HALL_POLL_HZ,
    HALL_PULL_UP,
    HALL_SENSOR_PIN,
    HALL_STABLE_SAMPLES,
    IDLE_LED_PIN,
    MAIN_LOOP_SLEEP,
    MEASURING_LED_BLINK_INTERVAL,
    MEASURING_LED_PIN,
    POWER_BUTTON_PIN,
    USB_CHECK_INTERVAL,
    USB_COPY_ANY,
    USB_COPY_LED_BLINK_INTERVAL,
    USB_COPY_LED_PIN,
)
from leds import CopyLED, IdleLED, MeasuringLED
from sensors import Accelerometer, HallSensor
from state_machine import StateMachine


class MeasurementSystem:
    """Main vibration measurement system coordinator."""

    CSV_FIELDNAMES = ["timestamp", "x", "y", "z", "spin_count"]

    def __init__(self):
        """Initialize the measurement system."""
        print("=" * 60)
        print("Streckenpruefer_GFA")
        print("=" * 60)
        print()

        self.state_machine = StateMachine()

        print("Initializing accelerometer...")
        self.accelerometer = Accelerometer(
            i2c_address=ACCELEROMETER_I2C_ADDRESS,
            bus=ACCELEROMETER_I2C_BUS,
            auto_detect=ACCELEROMETER_AUTO_DETECT,
            int1_gpio=ACCELEROMETER_INT1_GPIO,
            int2_gpio=ACCELEROMETER_INT2_GPIO,
            use_int1=ACCELEROMETER_USE_INT1,
            int_active_low=ACCELEROMETER_INT_ACTIVE_LOW,
            odr_hz=ACCELEROMETER_ODR_HZ,
            full_scale_g=ACCELEROMETER_FULL_SCALE_G,
            threshold_mps2=ACCELEROMETER_THRESHOLD_MPS2,
            transient_count=ACCELEROMETER_TRANSIENT_COUNT,
            dead_time_s=ACCELEROMETER_DEAD_TIME_S,
        )
        print()
        self.accelerometer.set_measurement_enabled(False)
        self.hall_sensor = None
        if HALL_ENABLED:
            print("Initializing Hall sensor...")
            self.hall_sensor = HallSensor(
                pin=HALL_SENSOR_PIN,
                pull_up=HALL_PULL_UP,
                poll_hz=HALL_POLL_HZ,
                stable_samples=HALL_STABLE_SAMPLES,
            )
            print()
            self.hall_sensor.reset_count()

        print("Initializing buttons...")
        self.begin_button = BeginButton(pin=BEGIN_BUTTON_PIN)
        self.begin_button.set_callback(self.on_begin_button_pressed)

        self.power_button = PowerButton(pin=POWER_BUTTON_PIN)
        self.power_button.set_shutdown_callback(self.on_shutdown)
        print()

        print("Initializing LEDs...")
        self.idle_led = IdleLED(pin=IDLE_LED_PIN)
        self.measuring_led = MeasuringLED(
            pin=MEASURING_LED_PIN,
            blink_interval=MEASURING_LED_BLINK_INTERVAL,
        )
        self.idle_led.turn_on()
        self.usb_copy_led = CopyLED(
            pin=USB_COPY_LED_PIN,
            blink_interval=USB_COPY_LED_BLINK_INTERVAL,
        )
        self.usb_copy_led.set_idle()
        print()

        self.running = True
        self.readings = []
        self.csv_output_path = CSV_OUTPUT_PATH
        self.usb_copy_any = USB_COPY_ANY
        self.usb_seen_mounts = set()
        self.last_usb_check_time = 0

    def _initialize_csv_file(self):
        """Create or overwrite the CSV at measurement start."""
        try:
            with open(self.csv_output_path, "w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=self.CSV_FIELDNAMES)
                writer.writeheader()
        except Exception as e:
            print(f"[CSV] Failed to initialize CSV file: {e}")

    def _append_reading_to_csv(self, reading):
        """Append a single reading during active measurement."""
        try:
            with open(self.csv_output_path, "a", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=self.CSV_FIELDNAMES)
                writer.writerow(reading)
        except Exception as e:
            print(f"[CSV] Failed to append reading: {e}")

    def on_begin_button_pressed(self):
     self.state_machine.toggle_measurement()

     if self.state_machine.is_measuring():
        self.readings.clear()
        self.accelerometer.clear_events()
        self.accelerometer.set_measurement_enabled(True)
        self._initialize_csv_file()
        if self.hall_sensor:
            self.hall_sensor.reset_count()
        self.idle_led.turn_off()
     else:
        self.accelerometer.set_measurement_enabled(False)
        self.measuring_led.turn_off()
        self.idle_led.turn_on()

    def on_shutdown(self):
     self.accelerometer.set_measurement_enabled(False)

     if self.state_machine.is_measuring():
        self.state_machine.stop_measurement()

     self.measuring_led.turn_off()
     self.idle_led.turn_on()
     self.save_readings_to_csv()

     print("\n[POWER] Measurement stopped. Returned to IDLE.")

    def read_vibration(self):
    
     try:
        events = self.accelerometer.drain_events()
        for event in events:
            reading = {
                "timestamp": event["timestamp"],
                "x": event["x"],
                "y": event["y"],
                "z": event["z"],
                "spin_count": self.hall_sensor.get_count() if self.hall_sensor else 0,
            }
            self.readings.append(reading)
            self._append_reading_to_csv(reading)
     except Exception as e:
        print(f"[ERROR] Failed to read accelerometer events: {e}")

    def _is_removable_mount(self, device, fstype, mount_point):
        if not device.startswith(("/dev/sd", "/dev/mmcblk")):
            return False
        if fstype not in {"vfat", "exfat", "ext4"}:
            return False
        return mount_point.startswith("/media") or mount_point.startswith("/run/media")

    def _scan_usb_mounts(self):
        """Return all mounted USB paths under /media and /run/media."""
        if not self.usb_copy_any:
            return []

        mounts = set()
        try:
            with open("/proc/mounts", "r") as mounts_file:
                for line in mounts_file:
                    parts = line.split()
                    if len(parts) < 3:
                        continue
                    device, mount_point, fstype = parts[0], parts[1], parts[2]
                    if self._is_removable_mount(device, fstype, mount_point):
                        mounts.add(mount_point)
        except Exception as e:
            print(f"[USB] Failed to scan mounts: {e}")
        return sorted(mounts)

    def _build_usb_csv_path(self, mount_path):
        """Build a timestamped CSV path on the USB drive."""
        base = os.path.splitext(os.path.basename(self.csv_output_path))[0]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(mount_path, f"{base}_{stamp}.csv")

    def _copy_csv_to_mounts(self, mount_paths):
        """Copy the latest CSV to one or more USB mount paths."""
        if not self.readings:
            print("[USB] No readings to copy yet.")
            return

        self.usb_copy_led.set_copying()
        self.save_readings_to_csv()

        success = False
        for mount_path in mount_paths:
            try:
                usb_csv_path = self._build_usb_csv_path(mount_path)
                shutil.copy2(self.csv_output_path, usb_csv_path)
                print(f"[USB] Copied CSV to {usb_csv_path}")
                success = True
            except Exception as e:
                print(f"[USB] Copy failed for {mount_path}: {e}")

        if success:
            self.usb_copy_led.set_copied()
        else:
            self.usb_copy_led.set_idle()
            print("[USB] No copies succeeded.")

    def _check_usb_copy(self):
        """Detect USB insertion or removal and copy CSV when inserted."""
        mounts = set(self._scan_usb_mounts())
        new_mounts = mounts - self.usb_seen_mounts

        if new_mounts:
            self._copy_csv_to_mounts(sorted(new_mounts))

        self.usb_seen_mounts = mounts
        if mounts:
            self.usb_copy_led.set_copied()
        else:
            self.usb_copy_led.set_idle()

    def run(self):
        """Main application loop."""
        print("System ready. Press BEGIN button to start measuring.")
        print("Hold POWER button for 2+ seconds to stop and save.")
        print("-" * 60)
        print()

        try:
            while self.running:
                self.begin_button.check_press()
                self.power_button.check_hold()

                if self.state_machine.is_measuring():
                    self.measuring_led.update()
                    self.read_vibration()
                else:
                    self.accelerometer.clear_events()

                self.usb_copy_led.update()

                current_time = time.time()
                if current_time - self.last_usb_check_time >= USB_CHECK_INTERVAL:
                    self.last_usb_check_time = current_time
                    self._check_usb_copy()

                time.sleep(MAIN_LOOP_SLEEP)

        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received.")
            self.on_shutdown()

        finally:
            self.cleanup()

    def save_readings_to_csv(self):
        """Write all readings to the CSV file."""
        if not self.readings:
            print("[CSV] No readings to save.")
            return

        try:
            with open(self.csv_output_path, "w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=self.CSV_FIELDNAMES)
                writer.writeheader()
                writer.writerows(self.readings)
            print(f"[CSV] Saved {len(self.readings)} readings to {self.csv_output_path}")
        except Exception as e:
            print(f"[CSV] Failed to write CSV file: {e}")

    def cleanup(self):
        """Clean up GPIO and other resources."""
        print("Cleaning up...")
        if self.hall_sensor:
            self.hall_sensor.cleanup()
        if self.accelerometer:
            self.accelerometer.cleanup()
        try:
            import RPi.GPIO as GPIO

            GPIO.cleanup()
            print("GPIO cleanup complete.")
        except Exception:
            pass

        self.save_readings_to_csv()

        print("=" * 60)
        print("System shutdown complete.")
        print("=" * 60)


def main():
    """Entry point."""
    system = MeasurementSystem()
    system.run()


if __name__ == "__main__":
    main()
