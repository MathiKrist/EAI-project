#!/usr/bin/env python3.5

import ev3dev.ev3 as ev3
from time import sleep

# --- Setup ---
btn = ev3.Button()

# Motors
mA = ev3.LargeMotor('outD') # Left motor
mB = ev3.LargeMotor('outA') # Right motor

# Sensors
left_sensor = ev3.ColorSensor('in1')   # Left sensor
right_sensor = ev3.ColorSensor('in4')  # Right sensor

left_sensor.mode = 'COL-REFLECT'
right_sensor.mode = 'COL-REFLECT'

assert left_sensor.connected, "Left Color Sensor not connected"
assert right_sensor.connected, "Right Color Sensor not connected"

# Parameters
BASE_SPEED = -20     # % duty cycle (negative for reverse)
KP = 5.5            # Proportional gain (tune this!)
KI = 0.05            # Integral gain (tune this!)
DELAY = 0.05         # Loop delay

# Calculate baseline (assume both sensors see the same surface)
baseline = (left_sensor.value() + right_sensor.value()) / 2

# Start motors
mA.run_direct()
mB.run_direct()

integral = 0  # Initialize integral term

# --- Main loop ---
while True:
    if btn.any():
        ev3.Sound.beep().wait()
        mA.duty_cycle_sp = 0
        mB.duty_cycle_sp = 0
        exit()

    # Read reflected light (0=black, 100=white)
    left_val = left_sensor.value()
    right_val = right_sensor.value()

    # Error = difference from baseline for each sensor
    error = (left_val - baseline) - (right_val - baseline)

    # Update integral
    integral += error * DELAY

    # Correction = proportional + integral control
    correction = KP * error + KI * integral

    # Adjust motor speeds
    mA.duty_cycle_sp = max(min(BASE_SPEED + correction, 100), -100)
    mB.duty_cycle_sp = max(min(BASE_SPEED - correction, 100), -100)

    # --- Log values ---
    print("Left:", left_val, " Right:", right_val, " Error:", error, " Corr:", correction, " Integral:", integral)

    sleep(DELAY)
