import os
os.system("sudo pigpiod")
import socket
import struct
import pigpio
from threading import Thread
import time
import compass_i2c
import gps_serial
from PIDController import PID
import accel_gyro_i2c
import math
time.sleep(1)

PID_ROLL = PID(0.5, 0.1, 0.01)
PID_PITCH = PID(0.5, 0.1, 0.01)
PID_YAW = PID(0.5,0.1,0.01)

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

sensitivity_throttle = 0.08
sensitivity = 0.1
deadzone = 0.09
#GPS = gps_serial.GPS()
#COMPASS = compass_i2c.Compass()
MAX_MOTOR_DIFF = 0.05
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
        ESC.set_speed(0)
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

armed = False
calibrated = False
throttle = 0.0
kill = False
arm = False
calibrate = False
translate_lr = 0.0
translate_ud = 0.0
translate_fb = 0.0
yaw = 0.0


def ControlThread():
    global running, kill, arm, calibrate, translate_lr, translate_ud, translate_fb, yaw, armed, throttle
    while running:
        controllerData = []
        while(controllerData == []):
            try:
                controllerData = recieveControllerData()
            except socket.timeout:
                print("WARNING: No Control Data.")
                if(armed):
                    print("Throttling down: "+str(throttle))
                    if(throttle > 0):
                        throttle -= 0.2 if(throttle <= 0.5) else 0.1
                        for esc in ESC_Array:
                            esc.set_speed(throttle)

                    else:
                        armed = False
                        throttle = 0
            except KeyboardInterrupt:
                running = False
        kill =          controllerData[0]
        arm =           controllerData[1]
        calibrate =     controllerData[2]
        translate_lr =  controllerData[3]
        translate_ud =  controllerData[4]
        translate_fb =  controllerData[5]
        yaw =           controllerData[6]

running = True

hover_throttle = 0.5



controlThread = Thread(target=ControlThread)
controlThread.start()

try:
    while running:

        pitch = accel_gyro_i2c.get_pitch() + 2.35
        roll = accel_gyro_i2c.get_roll() + 2.35
        delta_yaw = accel_gyro_i2c.get_yaw() - yaw
        yaw = delta_yaw + yaw

        direction = -1
        magnitude = 0
        if(abs(roll) > 1 or abs(pitch) > 1):
            y = math.sin(roll / 57.2958)
            x = math.sin(pitch / 57.2958)
            direction = (180 * math.atan(y/x) / math.pi) if (abs(x) > 0) else (90 if roll > 0 else -90)
            direction += 180 if (pitch < 0 ) else 0
            direction += 360 if(direction < 0) else 0
            magnitude = math.sqrt(x**2 + y**2)

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

            PID_ROLL.setTarget(translate_lr / 3 if abs(translate_lr) > deadzone else 0)
            delta = max(min(PID_ROLL.getDelta(roll) * sensitivity, MAX_MOTOR_DIFF), -1 * MAX_MOTOR_DIFF)
            ESC_Speeds[0] += delta
            ESC_Speeds[1] -= delta
            ESC_Speeds[2] -= delta
            ESC_Speeds[3] += delta
            PID_PITCH.setTarget(translate_fb / 3 if abs(translate_fb) > deadzone else 0)
            delta = max(min(PID_PITCH.getDelta(pitch) * sensitivity, MAX_MOTOR_DIFF), -1 * MAX_MOTOR_DIFF)
            ESC_Speeds[0] -= delta
            ESC_Speeds[1] -= delta
            ESC_Speeds[2] += delta
            ESC_Speeds[3] += delta
            PID_YAW.setTarget(0)
            delta = 0 #max(min(PID_YAW.getDelta(delta_yaw) * sensitivity, MAX_MOTOR_DIFF), -1 * MAX_MOTOR_DIFF)
            ESC_Speeds[0] += delta
            ESC_Speeds[1] -= delta
            ESC_Speeds[2] += delta
            ESC_Speeds[3] -= delta
            ESC_Speeds = [minMaxRange(throttle + ESC_Speeds[i]) for i in range(4)]
            for s in range(4):
                ESC_Array[s].set_speed(ESC_Speeds[s])

        if(EPOCH % 20 == 0):
            #position = GPS.getPosition()


            #print("Position: Lat=" + str(position[0]) + " Lon:"+ str(position[1]))
            #print("Heading: " + str(COMPASS.getHeading()))
            print("pitch: " + str(pitch))
            print("roll: " + str(roll))
            print("yaw: " + str(yaw))
            print("DIRECTION: " + str(direction))
            print("MAGNITUDE: " + str(magnitude))
            print("Speeds:")
            print(ESC_Speeds)

        EPOCH+=1
except Exception as e:
    print(str(e))
    Kill()
    #GPS.stop()
    #COMPASS.stop()





