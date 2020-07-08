import time
from collections import deque
import accel_gyro_i2c
import sys
import os

class PID():
  def __init__(self,p,i,d):
    self.P = p
    self.I = i
    self.D = d
    self.Cp = 0.0
    self.Ci = 0.0
    self.Cd = 0.0
    self.target = None
    self.lastDelta = None
  def setTarget(self, target):
    self.target = target
  def getDelta(self, current):
    error = self.target - current
    t = time.time()
    if(self.lastDelta == None):
      self.Cp = error
      self.Ci = 0.0
      self.Cd = 0.0
    else:
      dt = (t - self.lastDelta)
      self.Cd = (error - self.Cp) / dt
      self.Cp = error
      self.Ci += error * dt

    self.lastDelta = t
    u = self.P * self.Cp + self.I * self.Ci + self.D * self.Cd


    return u

if __name__ == "__main__":

  pidLR = PID(0.5,1,0.1)
  if(len(sys.argv) == 4):
    print("Using arguments as variables P, I, D")
    pid = PID(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
  pidLR.setTarget(0)
  while(1):
    gyro = accel_gyro_i2c.get_acc_y()
    delta = pidLR.getDelta(gyro)
    text = ": "+ '\033[94m' + '▉' * int(abs(delta) * 10)
    print("gyro: " + str(round(gyro,2)) +", delta: " + str(round(delta,2)) + text)

    time.sleep(0.01)

