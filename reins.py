import board
import busio
import time
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS
from threading import Thread, Lock
import sys

sys.path.append('unitree_legged_sdk/lib/python/arm64')
import robot_interface as sdk

HIGHLEVEL = 0xee

right_relaxed = 1145
right_pulled = 4100
left_relaxed = 1122
left_pulled = 4206

rate = 500       # 500hz
dt = 1.0 / rate  # time between commands

class B1Control:
  def Update(self):
    while True:
      if self.stop_thread:
        break
      self.lock.acquire()
      self.udp.SetSend(self.cmd)
      self.lock.release()
      self.udp.Send()
      time.sleep(dt)

  def Start(self):
    print("Standing...")
    self.lock.acquire()
    self.cmd.mode = 6 # stand
    self.lock.release()
    time.sleep(4.0)

    print("Initializing...")
    self.lock.acquire()
    self.cmd.mode = 1
    self.lock.release()
    time.sleep(1.0)

    print("Enabling Walk Control...")
    self.lock.acquire()
    self.cmd.mode = 2 # Walk
    self.cmd.gaitType = 1 # Trot
    self.cmd.speedLevel = 0 # Default
    self.lock.release()
    time.sleep(1.0)

    print("Done")

  def Stop(self):
    self.stop_thread = True

  def SetCmdVel(x, yaw):
    self.lock.acquire()
    self.x = x
    self.yaw = yaw
    self.lock.release()

  def __init__(self):
    self.x = 0.0
    self.yaw = 0.0

    self.udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.220", 8082)

    self.cmd = sdk.HighCmd()
    self.state = sdk.HighState()
    self.udp.InitCmdData(self.cmd)

    self.cmd.mode = 0
    self.cmd.gaitType = 0
    self.cmd.speedLevel = 0
    self.cmd.footRaiseHeight = 0
    self.cmd.bodyHeight = 0
    self.cmd.euler = [0, 0, 0]
    self.cmd.velocity = [0, 0]
    self.cmd.yawSpeed = 0.0
    self.cmd.reserve = 0

    self.stop_thread = False
    self.lock = Lock()
    self.thread = Thread(target=self.Update, args=())
    self.thread.daemon = True
    self.thread.start()

    print("B1 Control Started")

def flip(in_val):
  return clamp ( ( in_val - 1.0 ) * -1.0 )

def clamp(val, min_val=0.0, max_val=1.0):
  return max(min_val, min(val, max_val))

def main():
  i2c = busio.I2C(board.SCL, board.SDA)

  ads = ADS.ADS1115(i2c)
  right = AnalogIn(ads, ADS.P0)
  left = AnalogIn(ads, ADS.P1)

  B1 = B1Control()
  B1.Start()

#  for i in range(1000):
#    r_val = flip( (right.value - right_relaxed) / (right_pulled - right_relaxed) )
#    l_val = flip( (left.value - left_relaxed) / (left_pulled - left_relaxed) )
#
#    yaw_scale = r_val - l_val
#    x_scale = min(r_val, l_val)
#
#    print(x_scale, yaw_scale)
#    time.sleep(0.1)

  print("done!")

main()
