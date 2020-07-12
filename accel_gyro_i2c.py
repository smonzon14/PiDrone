'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import time

import smbus			#import SMBus module of I2C
from time import sleep          #import
import math
from threading import Thread
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

acc_x = 0
acc_y = 0
acc_z = 0

gyro_x = 0
gyro_y = 0
gyro_z = 0

gyro_offsets = [0, 0, 0]
acc_offsets = [0,0,0]

pitch = 0
roll = 0



def MPU_Init():
  #write to sample rate register
  bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

  #Write to power management register
  bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

  #Write to Configuration register
  bus.write_byte_data(Device_Address, CONFIG, 0)

  #Write to Gyro configuration register
  bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

  #Write to interrupt enable register
  bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
  #Accelero and Gyro value are 16-bit
  high = bus.read_byte_data(Device_Address, addr)
  low = bus.read_byte_data(Device_Address, addr+1)

  #concatenate higher and lower value
  value = ((high << 8) | low)

  #to get signed value from mpu6050
  if(value > 32768):
    value = value - 65536
  return value



def calibrate_gyro():

  for cal_int in range(2000):
    if(cal_int % 125 == 0):
      print(".")
    gyro_offsets[0] += get_gyro_x()
    gyro_offsets[1] += get_acc_y()
    gyro_offsets[2] += get_gyro_z()
    sleep(0.000003)
  gyro_offsets[0] /= 2000
  gyro_offsets[1] /= 2000
  gyro_offsets[2] /= 2000

def update():
  global pitch, roll
  pitch_gyro = 0
  roll_gyro = 0
  lastUpdate = time.time()
  set_gyro = False
  while 1:
    gyro_x = read_raw_data(GYRO_XOUT_H) - gyro_offsets[0]
    gyro_y = read_raw_data(GYRO_YOUT_H) - gyro_offsets[1]
    gyro_z = read_raw_data(GYRO_ZOUT_H) - gyro_offsets[2]

    acc_x = read_raw_data(ACCEL_XOUT_H) - acc_offsets[0]
    acc_y = read_raw_data(ACCEL_YOUT_H) - acc_offsets[1]
    acc_z = read_raw_data(ACCEL_ZOUT_H) - acc_offsets[2]

    pitch_gyro += gyro_y * 0.0000611
    roll_gyro += gyro_x * 0.0000611
    pitch_gyro += roll_gyro * math.sin(gyro_z * 0.000001066)
    roll_gyro -= pitch_gyro * math.sin(gyro_z * 0.000001066)

    acc_total_vector = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    pitch_acc = math.asin(acc_x/acc_total_vector) * -57.296
    roll_acc = math.asin(acc_y/acc_total_vector) * 57.296



    if(set_gyro):
      pitch_gyro = pitch_gyro * 0.9996 + pitch_acc * 0.0004
      roll_gyro = roll_gyro * 0.9996 + roll_acc * 0.0004
    else:
      pitch_gyro = pitch_acc
      roll_gyro = roll_acc
      set_gyro = True

    pitch = pitch * 0.9 + pitch_gyro * 0.1
    roll = roll * 0.9 + roll_gyro * 0.1

    while(time.time()-lastUpdate < 0.004): pass
    lastUpdate = time.time()


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()
updateThread = Thread(target=update)
updateThread.start()

def get_acc_x():
  return read_raw_data(ACCEL_XOUT_H)/16384.0
def get_acc_y():
  return read_raw_data(ACCEL_YOUT_H)/16384.0
def get_acc_z():
  return read_raw_data(ACCEL_ZOUT_H)/16384.0

def get_gyro_x():
  return read_raw_data(GYRO_XOUT_H)/131.0
def get_gyro_y():
  return read_raw_data(GYRO_YOUT_H)/131.0
def get_gyro_z():
  return read_raw_data(GYRO_ZOUT_H)/131.0


def get_pitch():
  return pitch

def get_roll():
  return roll





if __name__ == "__main__":
  print (" Reading Data of Gyroscope and Accelerometer")
  calibrate_gyro()
  print(gyro_offsets)
  while True:

    pitch = get_pitch()
    roll = get_roll()
    print("PITCH: " + str(pitch) + ", ROLL: " + str(roll))
    sleep(0.1)