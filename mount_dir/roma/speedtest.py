import time
import sys
from class1 import FourXFourBotControl
import math

FFBC = FourXFourBotControl()
s = float(input("Distance Input: "))
sp = float(input("Speed Input: "))
t = s / 1.26
print(round(t, 1), "s")
print("Preparing...")
time.sleep(2)
FFBC.set_wheels_frequency(-sp, sp, sp, -sp)
time.sleep(0.02)
FFBC.set_wheels_frequency(-sp, sp, sp, -sp)
time.sleep(0.02)
FFBC.set_wheels_frequency(-sp, sp, sp, -sp)
time.sleep(0.02)
FFBC.set_wheels_frequency(-sp, sp, sp, -sp)
print("Movement.")
time.sleep(round(t, 1))
FFBC.set_wheels_frequency(0, 0, 0, 0)
time.sleep(0.02)
FFBC.set_wheels_frequency(0, 0, 0, 0)
time.sleep(0.02)
FFBC.set_wheels_frequency(0, 0, 0, 0)
time.sleep(0.02)
FFBC.set_wheels_frequency(0, 0, 0, 0)
time.sleep(0.02)
FFBC.set_wheels_frequency(0, 0, 0, 0)
print("Finishing...")

