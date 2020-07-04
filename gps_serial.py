import serial
import time
from threading import Thread

class GPS():
  def __init__(self, serial_port="/dev/serial0"):
    self.serial_port = serial_port
    self.gps = serial.Serial(serial_port, baudrate = 9600, timeout = 0.5)
    self.running = True
    self.longitude = None
    self.latitude = None
    self.lastLock = None
    self.updateThread = Thread(target=self.__update, args=(True,))
    self.updateThread.start()

  def stop(self):
    self.gps.close()
    self.running = False

  def __update(self, verbose=False):
    while(self.running):
      data = self.gps.readline()
      data = data.decode("utf-8")

      message = data[0:6]

      if (message == "$GNRMC"):

        # GPRMC = Recommended minimum specific GPS/Transit data
        # Reading the GPS fix data is an alternative approach that also works
        parts = data.split(",")
        if parts[2] == 'V':
          # V = Warning, most likely, there are no satellites in view...
          print("GPS receiver warning")
        else:
          # Get the position data that was transmitted with the GPRMC message
          # In this example, I'm only interested in the longitude and latitude
          # for other values, that can be read, refer to: http://aprs.gids.nl/nmea/#rmc
          self.longitude = parts[5]
          self.latitude = parts[3]
          self.lastLock = time.time()
          if(verbose):
            print("["+str(self.lastLock)+"] GPS Location lock: lon = " + str(self.longitude) + ", lat = " + str(self.latitude))

if __name__ == "__main__":
  gps = GPS()
  while True:
    try:
      time.sleep(1)
    except KeyboardInterrupt:
      gps.stop()
      break
