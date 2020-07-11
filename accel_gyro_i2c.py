'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import smbus			#import SMBus module of I2C
from time import sleep          #import
import math
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

OFFSETS = [0.3, -0.11, -0.22, 0, 0, 0.1]


def get_acc_x():
  return read_raw_data(ACCEL_XOUT_H)/16384.0 + OFFSETS[3]
def get_acc_y():
  return read_raw_data(ACCEL_YOUT_H)/16384.0 + OFFSETS[4]
def get_acc_z():
  return read_raw_data(ACCEL_ZOUT_H)/16384.0 + OFFSETS[5]

def get_gyro_x():
  return read_raw_data(GYRO_XOUT_H)/131.0 + OFFSETS[0]
def get_gyro_y():
  return read_raw_data(GYRO_YOUT_H)/131.0 + OFFSETS[1]
def get_gyro_z():
  return read_raw_data(GYRO_ZOUT_H)/131.0 + OFFSETS[2]

def get_all():
  Ax = get_acc_x()
  Ay = get_acc_y()
  Az = get_acc_z()

  Gx = get_gyro_x()
  Gy = get_gyro_y()
  Gz = get_gyro_z()
  return Ax,Ay,Az,Gx,Gy,Gz

def get_pitch():
  return round(180 * math.atan2(Ax, math.sqrt(Ay**2 + Az**2))/math.pi, 2)

def get_roll():
  return -1 * round(180 * math.atan2(Ay, math.sqrt(Ax**2 + Az**2))/math.pi, 2)
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()



if __name__ == "__main__":
  print (" Reading Data of Gyroscope and Accelerometer")
  while True:

    pitch = get_pitch()
    roll = get_roll()
    print("PITCH: " + str(pitch) + ", ROLL: " + str(roll))
    sleep(0.1)