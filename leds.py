"""LED control for system state indication."""

import time


class LED:
    """Base class for LEDs."""
    
    def __init__(self, pin, name):
        """
        Initialize an LED.
        
        Args:
            pin: GPIO pin number for the LED
            name: Name of the LED (for logging)
        """
        self.pin = pin
        self.name = name
        self.GPIO = None
        self.is_on = False
        
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Start with LED off
            print(f"[{name}] LED initialized on GPIO {pin}")
        except ImportError:
            print(f"[{name}] Warning: RPi.GPIO not available, using simulated mode")
        except Exception as e:
            print(f"[{name}] Warning: Could not initialize - {e}")
    
    def turn_on(self):
        """Turn the LED on."""
        if self.GPIO is None:
            self.is_on = True
            return
        
        try:
            self.GPIO.output(self.pin, self.GPIO.HIGH)
            self.is_on = True
        except Exception as e:
            print(f"[{self.name}] Error turning on: {e}")
    
    def turn_off(self):
        """Turn the LED off."""
        if self.GPIO is None:
            self.is_on = False
            return
        
        try:
            self.GPIO.output(self.pin, self.GPIO.LOW)
            self.is_on = False
        except Exception as e:
            print(f"[{self.name}] Error turning off: {e}")


class IdleLED(LED):
    """
    IDLE state indicator LED.
    
    Stays lit while system is in IDLE state.
    """
    
    def __init__(self, pin=5, name="IDLE_LED"):
        """Initialize the IDLE LED on GPIO 5 by default."""
        super().__init__(pin, name)


class MeasuringLED(LED):
    """
    MEASURING state indicator LED.
    
    Blinks while system is in MEASURING state.
    """
    
    def __init__(self, pin=6, name="MEASURING_LED", blink_interval=0.5):
        """
        Initialize the MEASURING LED on GPIO 6 by default.
        
        Args:
            pin: GPIO pin number (default: 6)
            name: Name of the LED (default: "MEASURING_LED")
            blink_interval: Interval between blinks in seconds (default: 0.5)
        """
        super().__init__(pin, name)
        self.blink_interval = blink_interval
        self.last_blink_time = 0
    
    def update(self):
        """Update LED blinking state. Call this regularly in the main loop."""
        current_time = time.time()
        if current_time - self.last_blink_time >= self.blink_interval:
            # Toggle the LED
            if self.is_on:
                self.turn_off()
            else:
                self.turn_on()
            self.last_blink_time = current_time


class CopyLED(LED):
    """
    USB copy status LED.

    Blinks while copying and stays solid when a copy succeeds.
    """

    def __init__(self, pin=13, name="USB_COPY_LED", blink_interval=0.2):
        """Initialize the USB copy status LED."""
        super().__init__(pin, name)
        self.blink_interval = blink_interval
        self.last_blink_time = 0
        self.mode = "off"  # off | blinking | on

    def set_copying(self):
        """Set LED to blinking mode."""
        self.mode = "blinking"

    def set_copied(self):
        """Set LED to solid on."""
        self.mode = "on"
        self.turn_on()

    def set_idle(self):
        """Turn LED off."""
        self.mode = "off"
        self.turn_off()

    def update(self):
        """Update LED state based on current mode."""
        if self.mode == "blinking":
            current_time = time.time()
            if current_time - self.last_blink_time >= self.blink_interval:
                if self.is_on:
                    self.turn_off()
                else:
                    self.turn_on()
                self.last_blink_time = current_time
        elif self.mode == "on":
            if not self.is_on:
                self.turn_on()
        else:
            if self.is_on:
                self.turn_off()
