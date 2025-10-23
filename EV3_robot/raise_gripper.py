#!/usr/bin/env python3.5

import ev3dev.ev3 as ev3
from time import sleep

# Motor for cage
mC = ev3.LargeMotor('outB')

assert mC.connected, "Cage motor not connected"

# === PARAMETERS ===
RAISE_TIME = 2.0   # seconds — should match the lowering time, but tweak if needed

print("Raising cage...")
ev3.Sound.beep()

# Run the cage motor upward
mC.run_forever(speed_sp=-100)
sleep(RAISE_TIME)
mC.stop(stop_action="hold")

print("Cage raised. Adjust RAISE_TIME if needed.")
ev3.Sound.beep()
