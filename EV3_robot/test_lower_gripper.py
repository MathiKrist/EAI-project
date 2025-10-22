#!/usr/bin/env python3.5

import ev3dev.ev3 as ev3
from time import sleep

# Motor for cage
mC = ev3.MediumMotor('outC')

assert mC.connected, "Cage motor not connected"

# === PARAMETERS ===
LOWER_TIME = 1.0   # seconds — adjust this value while testing

print("Lowering cage...")
ev3.Sound.beep()

# Run the cage motor downward
mC.run_forever(speed_sp=-200)
sleep(LOWER_TIME)
mC.stop(stop_action="hold")

print("Cage lowered. Adjust LOWER_TIME if needed.")
ev3.Sound.beep()
