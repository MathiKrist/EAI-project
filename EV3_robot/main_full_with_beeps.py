#!/usr/bin/env python3.5
import ev3dev.ev3 as ev3
from time import sleep, time

# === SETUP ===
btn = ev3.Button()

# Motors
mA = ev3.LargeMotor('outD')  # Left drive motor
mB = ev3.LargeMotor('outA')  # Right drive motor
mC = ev3.MediumMotor('outC') # Cage motor (rear)

# Sensors
left_sensor = ev3.ColorSensor('in1')
right_sensor = ev3.ColorSensor('in4')
us = ev3.UltrasonicSensor('in2')

left_sensor.mode = 'COL-REFLECT'
right_sensor.mode = 'COL-REFLECT'
us.mode = 'US-DIST-CM'

assert left_sensor.connected and right_sensor.connected
assert us.connected and mC.connected

# === PARAMETERS ===
BASE_SPEED = -60
KP = 2
KI = 0.05
DELAY = 0.05

END_THRESHOLD = 70
END_CONFIRM_TIME = 1.5

SCAN_SPEED = 20
BACKWARD_SPEED = -25
STOP_DISTANCE = 5
CAGE_DOWN_TIME = 1.0
CAGE_UP_TIME = 1.0
SCAN_ANGLE = 90

# === HELPER FUNCTIONS ===
def stop_motors():
    mA.duty_cycle_sp = 0
    mB.duty_cycle_sp = 0

def drive(l, r):
    mA.duty_cycle_sp = l
    mB.duty_cycle_sp = r

def lower_cage():
    print("Lowering cage...")
    ev3.Sound.beep()
    mC.run_forever(speed_sp=-200)
    sleep(CAGE_DOWN_TIME)
    mC.stop(stop_action="hold")

def raise_cage():
    print("Raising cage...")
    ev3.Sound.beep()
    mC.run_forever(speed_sp=200)
    sleep(CAGE_UP_TIME)
    mC.stop(stop_action="hold")

def on_line():
    # True if either sensor sees dark (black/stipple)
    return left_sensor.value() < END_THRESHOLD or right_sensor.value() < END_THRESHOLD


# === PHASE 1: LINE FOLLOWING FORWARD ===
print("Starting line following...")
ev3.Sound.beep()
mA.run_direct()
mB.run_direct()

baseline = (left_sensor.value() + right_sensor.value()) / 2
integral = 0
white_since = None

while True:
    if btn.any():
        stop_motors()
        exit()

    left_val = left_sensor.value()
    right_val = right_sensor.value()
    error = (left_val - baseline) - (right_val - baseline)
    integral += error * DELAY
    correction = KP * error + KI * integral

    mA.duty_cycle_sp = max(min(BASE_SPEED - correction, 100), -100)
    mB.duty_cycle_sp = max(min(BASE_SPEED + correction, 100), -100)

    # End-of-line detection with persistence
    if left_val > END_THRESHOLD and right_val > END_THRESHOLD:
        if white_since is None:
            white_since = time()
        elif time() - white_since > END_CONFIRM_TIME:
            stop_motors()
            ev3.Sound.beep()
            print("End of track detected.")
            break
    else:
        white_since = None

    sleep(DELAY)


# === PHASE 2: SCAN FOR CAN ===
print("Scanning for can...")
ev3.Sound.beep()
mA.run_direct()
mB.run_direct()

best_distance = 255
best_angle = 0
angle = 0
direction = 1

for i in range(int(SCAN_ANGLE / 2)):
    drive(direction * SCAN_SPEED, -direction * SCAN_SPEED)
    dist = us.value() / 10.0
    if 3 < dist < best_distance:
        best_distance = dist
        best_angle = angle
    angle += direction * 2
    sleep(0.05)
    if abs(angle) >= SCAN_ANGLE / 2:
        direction *= -1

stop_motors()
ev3.Sound.beep()
print("Best distance:", best_distance, "cm at angle:", best_angle)


# === PHASE 3: TURN 180° BECAUSE GRIPPER IS AT BACK ===
print("Turning 180° to face can with rear...")
ev3.Sound.beep()
drive(25, -25)
sleep(3.0)  # adjust to fit your robot
stop_motors()


# === PHASE 4: APPROACH CAN (BACKWARD) ===
print("Reversing toward can...")
ev3.Sound.beep()
while True:
    dist = us.value() / 10.0
    print("Distance:", dist)
    if dist <= STOP_DISTANCE:
        stop_motors()
        ev3.Sound.beep()
        print("Can reached.")
        break
    else:
        drive(BACKWARD_SPEED, BACKWARD_SPEED)
    sleep(0.05)

lower_cage()
sleep(0.5)


# === PHASE 5: RETURN TO LINE ===
print("Dragging can back to line...")
ev3.Sound.beep()
drive(BACKWARD_SPEED, BACKWARD_SPEED)

# Drive until line is detected again (black/stipple)
while not on_line():
    if btn.any():
        stop_motors()
        exit()
    sleep(0.05)

stop_motors()
raise_cage()
ev3.Sound.beep()
print("Back on line. Returning home...")


# === PHASE 6: LINE FOLLOWING BACK ===
BASE_SPEED = 60  # forward now
ev3.Sound.beep()
integral = 0
white_since = None

while True:
    if btn.any():
        stop_motors()
        exit()

    left_val = left_sensor.value()
    right_val = right_sensor.value()
    error = (left_val - baseline) - (right_val - baseline)
    integral += error * DELAY
    correction = KP * error + KI * integral

    mA.duty_cycle_sp = max(min(BASE_SPEED - correction, 100), -100)
    mB.duty_cycle_sp = max(min(BASE_SPEED + correction, 100), -100)

    # Stop if both sensors detect white again (back at start)
    if left_val > END_THRESHOLD and right_val > END_THRESHOLD:
        if white_since is None:
            white_since = time()
        elif time() - white_since > END_CONFIRM_TIME:
            stop_motors()
            ev3.Sound.beep()
            print("Returned to start! Mission complete.")
            break
    else:
        white_since = None

    sleep(DELAY)

print("Mission complete.")
ev3.Sound.beep()
