"""Button handlers for BEGIN and POWER buttons."""

import time


class Button:
    """Base button class with debouncing."""

    def __init__(self, pin, name, pull_up=True):
        self.pin = pin
        self.name = name
        self.pull_up = pull_up
        self.GPIO = None
        self.callback = None

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            pull = GPIO.PUD_UP if pull_up else GPIO.PUD_DOWN
            GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
            print(f"[{name}] Button initialized on GPIO {pin}")
        except ImportError:
            print(f"[{name}] Warning: RPi.GPIO not available, using simulated mode")
        except Exception as e:
            print(f"[{name}] Warning: Could not initialize - {e}")

    def set_callback(self, callback):
        self.callback = callback

    def is_pressed(self):
        if self.GPIO is None:
            return False

        try:
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
        super().__init__(pin, name, pull_up=True)
        self.last_press_time = 0
        self.debounce_time = 0.3
        self._armed = True

    def check_press(self):
        pressed = self.is_pressed()

        if not pressed:
            self._armed = True
            return False

        current_time = time.time()
        if self._armed and (current_time - self.last_press_time > self.debounce_time):
            self.last_press_time = current_time
            self._armed = False
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
        super().__init__(pin, name, pull_up=True)
        self.hold_threshold = 2.0
        self.press_start_time = None
        self.shutdown_callback = None

    def set_shutdown_callback(self, callback):
        self.shutdown_callback = callback

    def check_hold(self):
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
            if self.press_start_time is not None:
                hold_time = time.time() - self.press_start_time
                if hold_time <= self.hold_threshold:
                    print(f"[{self.name}] Released after {round(hold_time, 2)}s (not long enough)")
                self.press_start_time = None

        return False