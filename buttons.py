"""Button handlers for BEGIN and POWER buttons."""

import time
import threading


class Button:
    """Base button class with debouncing."""
    
    def __init__(self, pin, name, pull_up=True):
        """
        Initialize a button.
        
        Args:
            pin: GPIO pin number
            name: Name of the button (for logging)
            pull_up: Whether to use pull-up resistor
        """
        self.pin = pin
        self.name = name
        self.pull_up = pull_up
        self.GPIO = None
        self.callback = None
        
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            pull = GPIO.PUD_UP if pull_up else GPIO.PUD_DOWN
            GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
            print(f"[{name}] Button initialized on GPIO {pin}")
        except ImportError:
            print(f"[{name}] Warning: RPi.GPIO not available, using simulated mode")
        except Exception as e:
            print(f"[{name}] Warning: Could not initialize - {e}")
    
    def set_callback(self, callback):
        """Set callback function to be called on button press."""
        self.callback = callback
    
    def is_pressed(self):
        """Check if button is currently pressed."""
        if self.GPIO is None:
            return False
        
        try:
            # Button is pressed when GPIO reads LOW (pull-up) or HIGH (pull-down)
            state = self.GPIO.input(self.pin)
            return state == 0 if self.pull_up else state == 1
        except Exception:
            return False


class BeginButton(Button):
    """
    BEGIN button: toggles between IDLE and MEASURING states.
    
    Single press: toggle measurement on/off
    """
    
    def __init__(self, pin=17, name="BEGIN_BUTTON"):
        """Initialize the BEGIN button on GPIO 17 by default."""
        super().__init__(pin, name, pull_up=True)
        self.last_press_time = 0
        self.debounce_time = 0.3  # 300ms debounce
    
    def check_press(self):
        """Check if button was pressed (with debouncing)."""
        if not self.is_pressed():
            return False
        
        current_time = time.time()
        if current_time - self.last_press_time > self.debounce_time:
            self.last_press_time = current_time
            print(f"[{self.name}] Pressed - toggling measurement")
            if self.callback:
                self.callback()
            return True
        
        return False


class PowerButton(Button):
    """
    POWER button: stops measurement and shuts down when held for >2 seconds.
    
    Hold for >2 seconds: stop measurement and shut down
    """
    
    def __init__(self, pin=27, name="POWER_BUTTON"):
        """Initialize the POWER button on GPIO 27 by default."""
        super().__init__(pin, name, pull_up=True)
        self.hold_threshold = 2.0  # 2 seconds
        self.press_start_time = None
        self.shutdown_callback = None
    
    def set_shutdown_callback(self, callback):
        """Set callback function to be called when shutdown is triggered."""
        self.shutdown_callback = callback
    
    def check_hold(self):
        """Check if button is being held long enough for shutdown."""
        if self.is_pressed():
            if self.press_start_time is None:
                self.press_start_time = time.time()
                print(f"[{self.name}] Pressed (hold for {self.hold_threshold}s to shutdown)")
            
            hold_time = time.time() - self.press_start_time
            if hold_time > self.hold_threshold:
                print(f"[{self.name}] Held for {round(hold_time, 2)}s - Stopping measurement...")
                if self.shutdown_callback:
                    self.shutdown_callback()
                self.press_start_time = None
                return True
        else:
            # Button released
            if self.press_start_time is not None:
                hold_time = time.time() - self.press_start_time
                if hold_time <= self.hold_threshold:
                    print(f"[{self.name}] Released after {round(hold_time, 2)}s (not long enough)")
                self.press_start_time = None
        
        return False
