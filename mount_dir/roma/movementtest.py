import time
import sys
from class1 import FourXFourBotControl
import math

FFBC = FourXFourBotControl()
s = float(input("Distance input: "))
t = s / 1.26
print(round(t, 1), "s")
print("Prepairing...")
time.sleep(2)
exec(open("move_forward.py").read())
time.sleep(0.02)
exec(open("move_forward.py").read())
time.sleep(0.02)
exec(open("move_forward.py").read())
time.sleep(0.02)
exec(open("move_forward.py").read())
print("Moving.")
time.sleep(round(t, 1))
exec(open("stop_wheels.py").read())
time.sleep(0.02)
exec(open("stop_wheels.py").read())
time.sleep(0.02)
exec(open("stop_wheels.py").read())
time.sleep(0.02)
exec(open("stop_wheels.py").read())
time.sleep(0.02)
exec(open("stop_wheels.py").read())
print("Finishing...")

# test drive for some time
# Speed is around 1.26 m/s (diametr 0.2м, wheels frequ 120)


