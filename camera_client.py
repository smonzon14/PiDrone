#Server prog

import socket
import numpy
import time
import matplotlib.pyplot as plt

UDP_IP=''
UDP_PORT = 999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

s=""

while True:
  data, addr = sock.recvfrom(46080)
  s+= data
  if len(s) == (46080*20):
    frame = numpy.fromstring (s, dtype=numpy.uint8)
    frame = frame.reshape(480,640,3)
    plt.imshow("frame",frame)
    plt.show()
    s=""