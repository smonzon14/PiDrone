import py_qmc5883l as qmc
import time
import math
from threading import Thread

class Compass():
  def __init__(self):
    self.compass = qmc.QMC5883L()
    self.compass.calibration = [[1.0291846390883186, 0.05995325526887818, 11.637433086701016],
                           [0.05995325526887818, 1.123160434037164, 942.1546833011146],
                           [0.0, 0.0, 1.0]]
    self.compass.declination = -14.23
    self.data = [None, None, None, None]
    self.running = True
    self.updateThread = None
    self.start()

  def start(self):
    self.running = True
    self.updateThread = Thread(target=self.__update)
    self.compass.mode_continuous()
    self.updateThread.start()

  def stop(self):
    self.compass.mode_standby()
    self.running = False
    self.updateThread.join()

  def __update(self):
    while(self.running):
      self.data = self.compass.get_data()


  def getHeading(self):
    x, y = self.data[0:2]
    if x is None or y is None:
      return None
    else:
      b = math.degrees(math.atan2(y, x))
      if b < 0:
        b += 360.0
      b += self.compass.get_declination()
      if b < 0.0:
        b += 360.0
      elif b >= 360.0:
        b -= 360.0
    return b

  def getTemperature(self):
    return self.data[3]

if __name__ == "__main__":
  compass = Compass()
  while True:
    try:
      print("[Compass] Heading=" + str(compass.getHeading()) + "Temp=" + str(compass.getTemperature()))
      time.sleep(1)
    except KeyboardInterrupt:
      compass.stop()
      break
