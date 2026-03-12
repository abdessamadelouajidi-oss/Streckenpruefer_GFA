"""Simple state machine with IDLE and MEASURING states."""

from enum import Enum


class State(Enum):
    """System states."""
    IDLE = "IDLE"
    MEASURING = "MEASURING"


class StateMachine:
    """Simple state machine for managing measurement states."""
    
    def __init__(self):
        """Initialize the state machine in IDLE state."""
        self.current_state = State.IDLE
    
    def toggle_measurement(self):
        """Toggle between IDLE and MEASURING states."""
        if self.current_state == State.IDLE:
            self.current_state = State.MEASURING
            print("[STATE] Transitioned to MEASURING")
        else:
            self.current_state = State.IDLE
            print("[STATE] Transitioned to IDLE")
    
    def stop_measurement(self):
        """Stop measurement and return to IDLE state."""
        if self.current_state == State.MEASURING:
            self.current_state = State.IDLE
            print("[STATE] Stopped measurement, returned to IDLE")
    
    def is_measuring(self):
        """Check if currently measuring."""
        return self.current_state == State.MEASURING
