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
    self.target = None
    self.errors = deque([0]*20)
    self.lastDelta = time.time()
  def setTarget(self, target):
    self.errors = deque([0]*20)
    self.target = target
  def getDelta(self, current):
    error = self.target - current
    t = time.time()
    u = self.P * error + self.I * sum(self.errors) + self.D * (error - self.errors[0])/(t - self.lastDelta)
    self.errors.rotate(1)
    self.errors[0] = error
    self.lastDelta = t
    return u

if __name__ == "__main__":

  pid = PID(1,0.1,0.1)
  if(len(sys.argv) == 4):
    print("Using arguments as variables P, I, D")
    pid = PID(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

  pid.setTarget(0)
  while(1):
    gyro = accel_gyro_i2c.get_acc_y()
    delta = pid.getDelta(gyro)
    text = ": "+ '\033[94m' + '▉' * int(abs(delta) * 10)
    print("gyro: " + str(round(gyro,2)) +", delta: " + str(round(delta,2)) + text)
    time.sleep(0.1)