#Server prog

import socket
import numpy
import time
import matplotlib.pyplot as plt
import array

UDP_IP='192.168.1.11'
UDP_PORT = 2001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

s=bytearray()

while True:
  data, addr = sock.recvfrom(46080)
  s.extend(data)
  print(len(s))
  if len(s) == (20*46080):
    frame = numpy.array(s, dtype=int)
    frame = frame.reshape(480,640,3)

    print(frame)
    plt.matshow(frame)
    plt.show()
    s=bytearray()