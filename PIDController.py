import time
from collections import deque
import accel_gyro_i2c
import sys

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
  if(len(sys.argv) == 3):
    print("Using arguments as variables P, I, D")
    pid = PID(*sys.argv)

  pid.setTarget(0)
  while(1):
    delta = pid.getDelta(accel_gyro_i2c.get_acc_y())
    print(delta)
    time.sleep(0.1)
