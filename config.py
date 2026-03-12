"""Configuration settings for the measurement system."""

# Button GPIO pins
BEGIN_BUTTON_PIN = 17      # GPIO 17 for BEGIN button
POWER_BUTTON_PIN = 27      # GPIO 27 for POWER button

# Accelerometer configuration (MMA8452Q transient interrupt mode)
ACCELEROMETER_I2C_BUS = 1
ACCELEROMETER_I2C_ADDRESS = 0x1C
ACCELEROMETER_AUTO_DETECT = True  # Try 0x1C and 0x1D if needed
ACCELEROMETER_INT1_GPIO = 26  # BCM 26 / physical pin 37
ACCELEROMETER_INT2_GPIO = 19  # BCM 19 / physical pin 35
ACCELEROMETER_USE_INT1 = True
ACCELEROMETER_INT_ACTIVE_LOW = True
ACCELEROMETER_ODR_HZ = 100
ACCELEROMETER_FULL_SCALE_G = 8
ACCELEROMETER_THRESHOLD_MPS2 = 10.0
ACCELEROMETER_TRANSIENT_COUNT = 1
ACCELEROMETER_DEAD_TIME_S = 0.20

# Hall sensor (spin counter)
HALL_ENABLED = True  # Set False to disable spin counting
HALL_SENSOR_PIN = 22  # GPIO pin for Hall sensor input
HALL_PULL_UP = True  # Use pull-up resistor if sensor is open-collector
HALL_POLL_HZ = 800  # Polling frequency for threaded Hall reader
HALL_STABLE_SAMPLES = 5  # Re-arm after N consecutive HIGH samples

# LED pins
IDLE_LED_PIN = 5        # GPIO 5 - lights up when in IDLE state
MEASURING_LED_PIN = 6   # GPIO 6 - blinks while measuring
MEASURING_LED_BLINK_INTERVAL = 0.5  # Blink every 0.5 seconds
USB_COPY_LED_PIN = 13   # GPIO 13 - indicates USB copy status
USB_COPY_LED_BLINK_INTERVAL = 0.2  # Blink while copying

# Main loop timing
MAIN_LOOP_SLEEP = 0.01  # Seconds between loop iterations

# CSV output
CSV_OUTPUT_PATH = "measurements.csv"  # Saved after shutdown

# USB copy settings
USB_COPY_ANY = True  # Copy to all mounted USB drives found under /media or /run/media
USB_CHECK_INTERVAL = 1.0  # Seconds between USB mount checks
