import serial

SERIAL_PORT = "/dev/serial0"
running = True
latitude = -1
longitude = -1
print("Application started!")
gps = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 0.5)
# In the NMEA message, the position gets transmitted as:
# DDMM.MMMMM, where DD denotes the degrees and MM.MMMMM denotes
# the minutes. However, I want to convert this format to the following:
# DD.MMMM. This method converts a transmitted string to the desired format
def formatDegreesMinutes(coordinates, digits):

  parts = coordinates.split(".")

  if (len(parts) != 2):
    return coordinates

  if (digits > 3 or digits < 2):
    return coordinates

  left = parts[0]
  right = parts[1]
  degrees = str(left[:digits])
  minutes = str(right[:3])

  return degrees + "." + minutes

# This method reads the data from the serial port, the GPS dongle is attached to,
# and then parses the NMEA messages it transmits.
# gps is the serial port, that's used to communicate with the GPS adapter
def getPositionData():
  global latitude, longitude
  data = gps.readline()
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
      longitude = formatDegreesMinutes(parts[5], 3)
      latitude = formatDegreesMinutes(parts[3], 2)

      print("Your position: lon = " + str(longitude) + ", lat = " + str(latitude))
  else:
    # Handle other NMEA messages and unsupported strings
    pass
  return latitude, longitude


if __name__ == "__main__":
  while running:
    try:
      getPositionData()
    except KeyboardInterrupt:
      running = False
      gps.close()
      print("Application closed!")
    except Exception as e:

      # You should do some error handling here...
      print("Application error! " + str(e))

