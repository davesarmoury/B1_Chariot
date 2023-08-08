import board
import busio
import time
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS

right_relaxed = 1145
right_pulled = 4100
left_relaxed = 1122
left_pulled = 4206

i2c = busio.I2C(board.SCL, board.SDA)
#i2c = busio.I2C(board.SCL_1, board.SDA_1)
ads = ADS.ADS1115(i2c)
right = AnalogIn(ads, ADS.P0)
left = AnalogIn(ads, ADS.P1)

def clamp(val, min_val=0.0, max_val=1.0):
  return max(min_val, min(val, max_val))

def flip(in_val):
  return clamp ( ( in_val - 1.0 ) * -1.0 )

for i in range(1000):
  r_val = (right.value - right_relaxed) / (right_pulled - right_relaxed)
  l_val = (left.value - left_relaxed) / (left_pulled - left_relaxed)
  print(flip(l_val), flip(r_val))
  time.sleep(0.5)

print("done!")