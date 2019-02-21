import matplotlib.pyplot as plt
import struct
import serial
import numpy as np

AMOUNT = 4
LENGTH = 4000
COUNTS = AMOUNT * LENGTH * 4

s = serial.Serial("COM5", 115200)
print("OK")
while s.inWaiting()<COUNTS:
    i=0
print(s.inWaiting())
raw = s.read(COUNTS)
dat = struct.unpack('i'*AMOUNT*LENGTH, raw)

time = np.linspace(0, (LENGTH-1)*0.005, LENGTH)
val = []
for i in range(AMOUNT):
    val.append(dat[i*LENGTH:(i+1)*LENGTH])

plt.plot(time, val[0], label="chassis-power")
plt.plot(time, val[1], label="buffer")
plt.plot(time, val[2], label="power")
plt.plot(time, val[3], label="output-percent")
plt.legend()
plt.show()
