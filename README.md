# Streckenpruefer_GFA

`Streckenpruefer_GFA` keeps the `v6` interaction model and logging flow, but switches the accelerometer path to MMA8452Q transient interrupt detection. Buttons, LEDs, Hall spin counting, CSV export, and USB auto-copy remain in place. The distance sensor is removed in this version.

## Features

- BEGIN button toggles IDLE and MEASURING states
- POWER button hold stops measurement and saves CSV
- IDLE, MEASURING, and USB copy LEDs
- MMA8452Q interrupt-based transient detection over I2C
- Hall sensor spin counter logged with each saved event
- CSV export to `measurements.csv`
- USB auto-copy support when a drive is inserted

## Project Structure

```
Streckenpruefer_GFA/
|- main.py
|- sensors.py
|- config.py
|- state_machine.py
|- buttons.py
|- leds.py
|- requirements.txt
`- README.md
```

## Hardware Configuration

### GPIO Controls

- BEGIN Button: GPIO 17
- POWER Button: GPIO 27
- IDLE LED: GPIO 5
- MEASURING LED: GPIO 6
- USB COPY LED: GPIO 13

### Accelerometer

- MMA8452Q I2C address: `0x1C` or `0x1D`
- INT1: GPIO 26 (default)
- INT2: GPIO 19
- Default interrupt route: `INT1`
- Default mode: active-low transient interrupt

### Hall Sensor

- Default Hall sensor pin: GPIO 22
- Use pull-up if the Hall sensor output is open-collector

## CSV Output

Each saved row contains:

- `timestamp`
- `x`
- `y`
- `z`
- `spin_count`

Rows are appended when the MMA8452Q raises a transient interrupt and the system is in MEASURING mode.

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
2. Review `config.py` and adjust the I2C address, interrupt pin, threshold, or Hall settings if needed.
3. Run:
   ```bash
   python main.py
   ```

## Notes

- `ACCELEROMETER_THRESHOLD_MPS2` controls the transient threshold.
- `ACCELEROMETER_DEAD_TIME_S` suppresses repeated triggers immediately after an event.
- `ACCELEROMETER_USE_INT1` selects whether the MMA8452Q interrupt is read from INT1 or INT2.
- The distance sensor path from `v6` is intentionally removed.
