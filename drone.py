
import socket
import struct
import pigpio
import os
import time
import py_qmc5883l
import gps_serial

def minMaxRange(val):
    return max(min(val, 1.0), 0.0)

def main():
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('',UDP_PORT))
    #os.system ("sudo pigpiod")
    #time.sleep(1)
    """
        (cw)              (ccw)
          12              13
             \\         //
               \\_____// 
                |  ^  |
                |_____|
               //     \\
             //         \\
          19              16
       (ccw)               (cw)
     
    """
    ESC_Pins = [12, 13, 16, 19]
    ESC_Array = [ESC(pin) for pin in ESC_Pins]
    ESC_Speeds = [0.0, 0.0, 0.0, 0.0]
    armed = False
    calibrated = False
    throttle = 0
    sensitivity_throttle = 0.01
    sensitivity = 0.01
    deadzone = 0.03
    compass = py_qmc5883l.QMC5883L()
    compass.mode_standby()
    while 1:
        s = []
        while(s == []):
            try:
                sock.settimeout(1)
                data, addr = sock.recvfrom(1024)
                s = list(struct.unpack('3?4d',data))
                print("Received: " + str(s))
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
                        compass.mode_standby()
                        Kill(ESC_Array)
                        print("Killed all motors")


        kill =          s[0]
        arm =           s[1]
        calibrate =     s[2]
        translate_lr =  s[3]
        translate_fb =  s[4]
        translate_ud =  s[5]
        yaw =           s[6]

        if(kill):
            while(throttle > 0):

                throttle -= 0.2 if(throttle <= 0.5) else 0.1
                for esc in ESC_Array:
                    esc.set_speed(throttle)
                time.sleep(1)
            armed = False
            throttle = 0
            compass.mode_standby()
            Kill(ESC_Array)
            print("Killed all motors")

        if(arm and not armed):
            compass.mode_continuous()
            Arm(ESC_Array)
            armed=True
        elif(calibrate and not armed and not calibrated):
            Calibrate(ESC_Array)
            calibrated = True
        elif(armed):
            gps_serial.getPositionData()
            if(abs(translate_ud) > deadzone):
                throttle += translate_ud * sensitivity_throttle
                throttle = max(min(throttle, 1.0), 0.0)
            if(abs(translate_lr) > deadzone):
                delta = translate_lr * sensitivity
                ESC_Speeds[0] += delta
                ESC_Speeds[1] -= delta
                ESC_Speeds[2] -= delta
                ESC_Speeds[4] += delta
            if(abs(translate_fb) > deadzone):
                delta = translate_fb * sensitivity
                ESC_Speeds[0] -= delta
                ESC_Speeds[1] -= delta
                ESC_Speeds[2] += delta
                ESC_Speeds[4] += delta
            if(abs(yaw) > deadzone):
                delta = yaw * sensitivity
                ESC_Speeds[0] -= delta
                ESC_Speeds[1] += delta
                ESC_Speeds[2] -= delta
                ESC_Speeds[4] += delta
            ESC_Speeds = [minMaxRange(throttle + ESC_Speeds[i]) for i in range(4)]
            for s in range(4):
                ESC_Array[s].set_speed(ESC_Speeds[s])


def Calibrate(ESC_Array):   #This is the auto calibration procedure of a normal ESC
    for ESC in ESC_Array:
        ESC.set_pwm(0)
    print("Disconnect the battery and press Enter")
    input()
    for ESC in ESC_Array:
        ESC.set_speed(1)
    print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
    input()
    for ESC in ESC_Array:
        ESC.set_speed(0)
    for i in range(12) :
        time.sleep(1)
        print(".\r")
    for ESC in ESC_Array:
        ESC.set_pwm(0)
    print("Almost there...")
    time.sleep(2)
    for ESC in ESC_Array:
        ESC.set_speed(0)
    print("Done.")
    time.sleep(1)

def Arm(ESC_Array): #This is the arming procedure of an ESC
    for ESC in ESC_Array:
        ESC.start()
    print("Connect the battery and press Enter")
    input()
    for ESC in ESC_Array:
        ESC.set_pwm(0)
    time.sleep(1)
    for ESC in ESC_Array:
        ESC.set_speed(1)
    time.sleep(1)
    for ESC in ESC_Array:
        ESC.set_speed(0)
    time.sleep(1)
    print("Armed and ready!")

def Kill(ESC_Array):
    for ESC in ESC_Array:
        ESC.stop()

class ESC():

    def __init__(self, pin):

        self.pin = pin #Connect the ESC in this GPIO pin
        self.max_value = 2000 #change this if your ESC's max value is different or leave it be
        self.min_value = 700  #change this if your ESC's min value is different or leave it be
        self.start()

    def set_pwm(self, pwm):
        self.pi.set_servo_pulsewidth(self.pin, pwm)

    def set_speed(self, speed):
        # speed as a decimal (percentage)
        self.set_pwm(self.min_value + (self.max_value - self.min_value) * speed)

    def stop(self):
        self.set_pwm(0)
        # for some reason self.pi.stop() creates an error

    def start(self):
        self.pi = pigpio.pi()
        self.set_pwm(0)

if __name__ == '__main__':
    main()



