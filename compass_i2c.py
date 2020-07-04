import py_qmc5883l as qmc
import time
import math
class Compass():
  def __init__(self):
    self.compass = py_qmc5883l.QMC5883L()
    self.compass.calibration = [[1.0291846390883186, 0.05995325526887818, 11.637433086701016],
                           [0.05995325526887818, 1.123160434037164, 942.1546833011146],
                           [0.0, 0.0, 1.0]]
    self.compass.declination = -14.23
    self.data = [None, None, None, None]


  def __update(self):
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