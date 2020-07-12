
import socket
import struct
import pigpio
import os
import time
import compass_i2c
import gps_serial
from PIDController import PID
import accel_gyro_i2c
import math

class ESC():

    def __init__(self, pin):

        self.pin = pin #Connect the ESC in this GPIO pin
        self.max_value = 2000 #change this if your ESC's max value is different or leave it be
        self.min_value = 1000  #change this if your ESC's min value is different or leave it be
        self.start()

    def set_pwm(self, pwm):
        self.pi.set_servo_pulsewidth(self.pin, pwm)

    def set_speed(self, speed):
        # speed as a decimal (percentage)
        self.set_pwm(self.min_value + (self.max_value - self.min_value) * speed)

    def stop(self):
        self.set_speed(0)
        self.set_pwm(0)
        # for some reason self.pi.stop() creates an error

    def start(self):
        self.pi = pigpio.pi()
        self.set_pwm(0)

UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('',UDP_PORT))
#os.system ("sudo pigpiod")
#time.sleep(1)
"""

    (cw)               (ccw)
      13       ^      12
         \\    |x   //
           \\_____// 
            |  ^  | _y_>
            |_(z)_|
           //     \\
         //         \\
      19              16
   (ccw)                (cw)
 
"""

ESC_Pins = [13, 12, 16, 19]
ESC_Array = [ESC(pin) for pin in ESC_Pins]
ESC_Speeds = [0.0, 0.0, 0.0, 0.0]
armed = False
calibrated = False
throttle = 0.0
sensitivity_throttle = 0.08
sensitivity = 0.001
deadzone = 0.09
stalling = False
stall_speed = 0.5
#GPS = gps_serial.GPS()
#COMPASS = compass_i2c.Compass()
MAX_MOTOR_DIFF = 0.15
EPOCH = 0

def minMaxRange(val):
    return max(min(val, 1.0), 0.0)

def Calibrate():   #This is the auto calibration procedure of a normal ESC

    print("Disconnect the battery and press Enter")
    input()
    for ESC in ESC_Array:
        ESC.start()
        ESC.set_speed(1)
    print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
    input()
    for ESC in ESC_Array:
        ESC.set_speed(0)
    print("Wait for long beep.")
    input()
    print("Done.")

def Arm(): #This is the arming procedure of an ESC
    print("ARMING")
    for ESC in ESC_Array:
        ESC.start()
    time.sleep(2)
    print("Armed and ready!")

def Kill():
    for ESC in ESC_Array:
        ESC.stop()

def recieveControllerData(timeout=0.5):
    sock.settimeout(timeout)
    data, addr = sock.recvfrom(1024)
    s = list(struct.unpack('3?4d',data))
    return s

running = True

hover_throttle = 0.5

PID_LR = PID(20.0,0.005,0.01)
PID_FB = PID(20.0,0.005,0.01)

try:
    while running:
        s = []
        while(s == []):
            try:
                s = recieveControllerData()
            except socket.timeout:
                print("WARNING: No Control Data.")
                if(armed):
                    print("Throttling down: "+throttle)
                    if(throttle > 0):
                        throttle -= 0.2 if(throttle <= 0.5) else 0.1
                        for esc in ESC_Array:
                            esc.set_speed(throttle)

                    else:
                        armed = False
                        throttle = 0
            except KeyboardInterrupt:
                running = False


        kill =          s[0]
        arm =           s[1]
        calibrate =     s[2]
        translate_lr =  s[3]
        translate_ud =  s[4]
        translate_fb =  s[5]
        yaw =           s[6]

        pitch = accel_gyro_i2c.get_pitch()
        roll = accel_gyro_i2c.get_roll()

        if(kill):
            while(throttle > 0):
                throttle -= 0.2 if(throttle <= 0.5) else 0.1
                for esc in ESC_Array:
                    esc.set_speed(throttle)
                time.sleep(1)
            armed = False
            throttle = 0
            Kill()

        if(arm and not armed):

            Arm()
            armed=True
        elif(calibrate and not armed and not calibrated):
            Calibrate()
            calibrated = True
        elif(armed):
            ESC_Speeds = [0.0, 0.0, 0.0, 0.0]
            if(abs(translate_ud) > deadzone):
                if(translate_ud > 0):
                    if(translate_ud > throttle):
                        throttle = translate_ud
                else:
                    throttle += translate_ud * sensitivity_throttle
                throttle = minMaxRange(throttle)

            PID_LR.setTarget(translate_lr/3 if abs(translate_lr) > deadzone else 0)
            delta = max(min(PID_LR.getDelta(roll) * sensitivity, MAX_MOTOR_DIFF), -1 * MAX_MOTOR_DIFF)
            if(EPOCH % 10 == 0):
                print(delta)
            ESC_Speeds[0] += delta
            ESC_Speeds[1] -= delta
            ESC_Speeds[2] -= delta
            ESC_Speeds[3] += delta
            PID_FB.setTarget(translate_fb/3 if abs(translate_fb) > deadzone else 0)
            delta = max(min(PID_FB.getDelta(pitch) * sensitivity, MAX_MOTOR_DIFF), -1 * MAX_MOTOR_DIFF)
            if(EPOCH % 10 == 0):
                print(delta)
            ESC_Speeds[0] -= delta
            ESC_Speeds[1] -= delta
            ESC_Speeds[2] += delta
            ESC_Speeds[3] += delta
            if abs(yaw) > deadzone:
                delta = yaw * sensitivity /10
                ESC_Speeds[0] -= delta
                ESC_Speeds[1] += delta
                ESC_Speeds[2] -= delta
                ESC_Speeds[3] += delta
            ESC_Speeds = [minMaxRange(throttle + ESC_Speeds[i]) for i in range(4)]
            for s in range(4):
                ESC_Array[s].set_speed(ESC_Speeds[s])

        if(EPOCH % 10 == 0):
            #position = GPS.getPosition()


            #print("Position: Lat=" + str(position[0]) + " Lon:"+ str(position[1]))
            #print("Heading: " + str(COMPASS.getHeading()))
            print("pitch: " + str(pitch))
            print("roll: " + str(roll))
            print("Speeds:")
            print(ESC_Speeds)

        EPOCH+=1
except Exception as e:
    print(str(e))
    Kill()
    #GPS.stop()
    #COMPASS.stop()





