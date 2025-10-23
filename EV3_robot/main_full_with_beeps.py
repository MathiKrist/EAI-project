#!/usr/bin/env python3.5
import ev3dev.ev3 as ev3
from time import sleep, time

btn = ev3.Button()

# === HARDWARE SETUP ===
mA = ev3.LargeMotor('outD')  # left drive
mB = ev3.LargeMotor('outA')  # right drive
mC = ev3.LargeMotor('outB')  # cage / gripper motor

right_sensor = ev3.ColorSensor('in1')
left_sensor  = ev3.ColorSensor('in4')
us = ev3.UltrasonicSensor('in3')

right_sensor.mode = 'COL-REFLECT'
left_sensor.mode  = 'COL-REFLECT'
us.mode = 'US-DIST-CM'

assert right_sensor.connected and left_sensor.connected and us.connected

# === PARAMETERS ===

## --- LINE FOLLOWING ---
BASE_SPEED = -25
KP = 5
KI = 0.05
SAMPLE_INTERVAL = 0.05
STABLE_TOLERANCE = 0.5
STABLE_TIME = 0.55
MIN_RUN_TIME = 2.0
BLACK_THRESHOLD = 20

## --- TURNING ---
TURN_SPEED = 20
TURN_STEP_TIME = 0.05
DEGREES_PER_STEP = 2
TURN_ANGLE = 220
IGNORE_BLACK_AFTER = 160

## --- SCANNING ---
SCAN_SPEED = 25
SCAN_ANGLE = 45

## --- CAN APPROACH / GRIPPER ---
STOP_DISTANCE = 4.4          # cm
DRIVE_SPEED = 30
GRIPPER_DOWN_TIME = 2.0      # seconds to lower cage

## --- RETURN PHASE ---
BACKWARD_SPEED = -25

# === HELPER FUNCTIONS ===
def stop_motors():
    mA.duty_cycle_sp = 0
    mB.duty_cycle_sp = 0

def drive(l, r):
    mA.duty_cycle_sp = l
    mB.duty_cycle_sp = r

def sees_black():
    return (left_sensor.value() < BLACK_THRESHOLD or
            right_sensor.value() < BLACK_THRESHOLD)

def lower_gripper():
    print("Lowering gripper...")
    ev3.Sound.beep()
    mC.run_forever(speed_sp=100)
    sleep(GRIPPER_DOWN_TIME)
    mC.stop(stop_action="hold")

# === PHASE 1: LINE FOLLOWING ===
def line_follow(stop_when_stable=True):
    """Follows the line. 
       If stop_when_stable=True, stops when sensors stable (used on outbound trip).
       If False, keeps following until EV3 button pressed (used on return)."""
    print("Starting line following...")
    ev3.Sound.beep()
    mA.run_direct()
    mB.run_direct()

    baseline = (left_sensor.value() + right_sensor.value()) / 2
    integral = 0
    last_left = left_sensor.value()
    last_right = right_sensor.value()
    last_change_left = time()
    last_change_right = time()
    start_time = time()

    while True:
        # Allow manual stop at any time
        if btn.any():
            stop_motors()
            print("Stopped by button.")
            exit()

        left_val = left_sensor.value()
        right_val = right_sensor.value()

        if abs(left_val - last_left) > STABLE_TOLERANCE:
            last_change_left = time()
            last_left = left_val
        if abs(right_val - last_right) > STABLE_TOLERANCE:
            last_change_right = time()
            last_right = right_val

        error = (left_val - baseline) - (right_val - baseline)
        integral += error * SAMPLE_INTERVAL
        correction = KP * error + KI * integral

        mA.duty_cycle_sp = max(min(BASE_SPEED - correction, 100), -100)
        mB.duty_cycle_sp = max(min(BASE_SPEED + correction, 100), -100)

        # --- only trigger stop when outbound ---
        if stop_when_stable:
            if ((time() - start_time) > MIN_RUN_TIME and
                (time() - last_change_left)  >= STABLE_TIME and
                (time() - last_change_right) >= STABLE_TIME):
                stop_motors()
                ev3.Sound.beep()
                print("Line following stable -> starting 180 degree turn.")
                return
        sleep(SAMPLE_INTERVAL)


# === PHASE 2: 180 DEGREE TURN WITH IGNORE AFTER 160 ===
def turn_180_and_check_line():
    print("Turning 180 degrees...")
    ev3.Sound.beep()
    mA.run_direct()
    mB.run_direct()
    angle = 0

    while angle < TURN_ANGLE:
        drive(TURN_SPEED, -TURN_SPEED)
        sleep(TURN_STEP_TIME)
        angle += DEGREES_PER_STEP

        # Only stop if black seen before 160 degrees
        if sees_black() and angle < IGNORE_BLACK_AFTER:
            stop_motors()
            ev3.Sound.beep()
            print("Black line detected before %.0f deg -> return to line following." % IGNORE_BLACK_AFTER)
            return False

    stop_motors()
    ev3.Sound.beep()
    print("180 degree turn complete.")
    return True

# === PHASE 3: SCAN FOR CAN ===
def scan_for_can():
    print("Scanning for can...")
    ev3.Sound.beep()
    mA.run_direct()
    mB.run_direct()

    best_distance = 255
    best_angle = 0
    current_angle = 0

    # Turn left
    while current_angle > -SCAN_ANGLE:
        drive(-SCAN_SPEED, SCAN_SPEED)
        sleep(TURN_STEP_TIME)
        current_angle -= DEGREES_PER_STEP
        dist = us.value() / 10.0
        if 3 < dist < best_distance:
            best_distance = dist
            best_angle = current_angle
    stop_motors()
    sleep(0.3)

    # Turn right through full range
    while current_angle < SCAN_ANGLE:
        drive(SCAN_SPEED, -SCAN_SPEED)
        sleep(TURN_STEP_TIME)
        current_angle += DEGREES_PER_STEP
        dist = us.value() / 10.0
        if 3 < dist < best_distance:
            best_distance = dist
            best_angle = current_angle
    stop_motors()
    sleep(0.3)

    print("Scan complete -> Closest object at %.1f cm, angle %.1f deg" %
          (best_distance, best_angle))
    ev3.Sound.beep()
    ev3.Sound.beep()

    # Turn to face the can
    current_angle = SCAN_ANGLE
    while abs(current_angle - best_angle) > 1:
        if current_angle > best_angle:
            drive(-SCAN_SPEED, SCAN_SPEED)
            current_angle -= DEGREES_PER_STEP
        else:
            drive(SCAN_SPEED, -SCAN_SPEED)
            current_angle += DEGREES_PER_STEP
        sleep(TURN_STEP_TIME)
    stop_motors()
    ev3.Sound.beep()
    print("Facing can at %.1f deg, distance %.1f cm." % (best_angle, best_distance))

    return best_distance, best_angle

# === PHASE 4: DRIVE TO CAN AND LOWER GRIPPER ===
def drive_to_can(distance):
    print("Approaching can...")
    ev3.Sound.beep()
    mA.run_direct()
    mB.run_direct()

    start_dist = us.value() / 10.0
    while True:
        dist = us.value() / 10.0
        print("Distance to can: %.1f cm" % dist)
        if dist <= STOP_DISTANCE:
            stop_motors()
            ev3.Sound.beep()
            print("Can reached (%.1f cm). Lowering gripper..." % dist)
            lower_gripper()
            break
        else:
            drive(DRIVE_SPEED, DRIVE_SPEED)
        sleep(0.05)

    stop_motors()
    ev3.Sound.beep()
    print("Can captured.")
    return start_dist

# === PHASE 5: RETURN TO LINE (REVISED) ===
def return_to_line(start_dist, best_angle):
    print("Backing up to scanning center...")
    ev3.Sound.beep()
    mA.run_direct()
    mB.run_direct()

    reverse_time = (start_dist - STOP_DISTANCE) / 10.0
    reverse_time = max(reverse_time, 1.5)
    drive(BACKWARD_SPEED, BACKWARD_SPEED)
    sleep(reverse_time)
    stop_motors()

    # Turn back by the same scan angle
    print("Turning %.1f degrees to face line direction..." % best_angle)
    ev3.Sound.beep()
    angle = 0
    while abs(angle) < abs(best_angle):
        if best_angle < 0:
            drive(TURN_SPEED, -TURN_SPEED)
        else:
            drive(-TURN_SPEED, TURN_SPEED)
        sleep(TURN_STEP_TIME)
        angle += DEGREES_PER_STEP
    stop_motors()
    ev3.Sound.beep()

    print("Searching for black line...")
    mA.run_direct()
    mB.run_direct()
    while not sees_black():
        drive(BACKWARD_SPEED, BACKWARD_SPEED)
        sleep(0.05)
    stop_motors()
    ev3.Sound.beep()
    print("Black line found. Ready to follow back.")
    return True

# === PHASE 6: LINE FOLLOW BACK TO START ===
def follow_back():
    print("Following line back to start... (press button to stop)")
    ev3.Sound.beep()
    # Keep driving backward indefinitely until user stops
    line_follow(stop_when_stable=False)


# === MAIN PROGRAM ===
while True:
    line_follow()
    turned = turn_180_and_check_line()
    if not turned:
        print("Resuming line following...")
        sleep(0.5)
        continue

    distance, best_angle = scan_for_can()
    start_dist = drive_to_can(distance)
    if return_to_line(start_dist, best_angle):
        follow_back()

    print("Mission complete.")
    ev3.Sound.beep()
    stop_motors()
    break
