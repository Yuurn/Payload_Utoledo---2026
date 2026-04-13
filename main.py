import machine
import time
#import buzzer
from machine import UART, Pin, I2C
#from bmp180 import BMP180
#from pca9685 import PCA9685Driver
from micropython_bmpxxx import bmpxxx
#from bno055 import *
#from servo import Servo
#import sensorLib
#import sdMount
import vfs
#global logfile
#logfile = "/sd/logfile.txt"

bmp_i2c = I2C(1, sda=Pin(8), scl=Pin(9))
imu_i2c = machine.I2C(1)

bmp = bmpxxx.BMP585(bmp_i2c)

press = bmp.pressure
temp = bmp.temperature

#imu = BNO055(imu_i2c)
calibrated = True

mmwave = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
time.sleep(2)

# Launch Detection
ground_alt = bmp.altitude
print(f"Starting altitude = {ground_alt:.2f} meters")

#a_x, a_y, a_z = imu.accel()
#print('Starting acceleration     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))

while True:
    print("Waiting for launch...")
    time.sleep(1)
    if not calibrated:
        calibrated = imu.calibrated()
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
        
    #detect = sensorLib.getTotalAccel() / 9.81
    #print(f"Total acceleration is {detect:.2f}.")
    
    new_alt = bmp.altitude
    print(f"New altitude detected as {new_alt:.2f} meters")
    
    alt_diff = abs(new_alt - ground_alt)
    print(f"Absolute difference in altitude is {alt_diff:.2f} meters")

    if alt_diff > 20: 
        print("Launch detected. Begin logging flight data...")
        break
    
    print()
    
# Flight Data Logging
armAccel = 1.2 #1.5
deployAccel = 0.75 #0.75
deployDelay = 1 #1

#4b,p,4e5,p,4b,p,4f#5,2p,4e5,2b5,8b
b = 493*2
e5 = 329*2
fs5 = 369*2
p = 1/40

def simplePredictor(alt1, alt2, t1, t2 , accel):
    #0^2 = curVel^2 + 2*(grav-accel)*s, solve for s
    curVel = ((alt2-alt1)/(t2-t1))
    #curVel = 25
    #return curVel 
    apTime = ((curVel)/(9.81)) +(t1-t2)
    #return apTime
    #v and a should be zero at apTime = 0
    appogee = ((9.81/2) * ((apTime*1.2)**2))  + alt1
    return appogee

def writeLogPacket(sTime, accel, alt, state, deployed, predicted):
    print("FUCK")
    return
    startCharacter = ""
    if state == "newLog":
        startCharacter = '\n'
    global logfile
    curSeconds = (time.time_ns()-sTime) / 1000000000
    logString = startCharacter+"t+"+str(int(curSeconds*100)/100)+"!"+str(abs(accel))+"!"+str(alt)+"!"+str(state)+"!"+str(deployed)+"!"+str(predicted)+"\n"
    logFile = open(logfile,"a")
    logFile.write(logString)
    logFile.close()

#import altitudePredictor
#input("waiting for arm")
#sensorLib.setStartingAltitude()
#print(sensorLib.getAltitude())
print("airbreaks armed")
index = 99

ledValue = 0

while True:
    index += 1
        
    #detect = (sensorLib.getTotalAccel()/9.81)
    #print(detect)
    #if index >= 25:
        #buzzer.tone(880,0.05)
        #index = 0
    startTime = time.time_ns()
    new_alt = bmp.altitude
    #print(f"New altitude detected as {new_alt:.2f} meters")
    alt_diff = abs(new_alt - ground_alt)
    if alt_diff > 3:
        #n[0] = (250,250,250)
        #n.write()
        #writeLogPacket(startTime,sensorLib.getTotalAccel()/9.81,sensorLib.getAltitude(),"newLog","no",0)
        #print("motor burn start detected, ",(detect))
        #while sensorLib.getTotalAccel()/9.81 >deployAccel:
            #writeLogPacket(startTime,sensorLib.getTotalAccel()/9.81,sensorLib.getAltitude(),"burn","no",0)
        #    time.sleep(0.01)
        #    continue
        prevTime = (time.time_ns()-startTime) / 1000000000
        prevAlt = (bmp.altitude - ground_alt)
        print("motor burnout detected")
        time.sleep(deployDelay)
        previousBaro = 0
        curAltitude = 0.01
        prevApogee = 0
        maxAlt = 0
        tStep = 0
        
        while curAltitude - prevApogee > -0.1 and maxAlt - curAltitude < 0.5:
            altitude = (bmp.altitude - ground_alt)
            #accel = sensorLib.getTotalAccel()/9.81
            #predicted = simplePredictor(altitude, prevAlt, (time.time_ns()-startTime)/1000000000, prevTime, accel)
            #writeLogPacket(startTime, accel, altitude, "ascending","yes",predicted)
            tStep += 1
            prevApogee = curAltitude
            time.sleep(0.1)
            curAltitude = (bmp.altitude - ground_alt)
            if curAltitude > maxAlt:
                maxAlt = curAltitude
            prevTime = (time.time_ns()-startTime) / 1000000000
            prevAlt = altitude
        print("apogee detected at ",curAltitude, "m, tStep = ",tStep)
        #writeLogPacket(startTime,sensorLib.getTotalAccel()/9.81,prevApogee,"apogee","no",predicted)
        break

# Payload Release
# 300 feet to deploy payload
# 300 ft in meters
DEPLOY_ALT = 91.4

# Velocity below 25 to deploy
# 25 ft/s in m/s
# DEPLOY_VEL = 7.62

def compute_velocity(alt1, alt2, t1, t2):
    if (t2 - t1) == 0:
        return 0
    return (alt2 - alt1) / (t2 - t1)

payload_deployed = False

prevAlt = (bmp.altitude - ground_alt)
prevTime = time.time()

servo1 = Servo(6)
servo2 = Servo(7)

while not payload_deployed:
    curAlt = (bmp.altitude - ground_alt)
    curTime = time.time()

    velocity = compute_velocity(prevAlt, curAlt, prevTime, curTime)

    print(f"Altitude: {curAlt:.2f} m")

    #writeLogPacket(startTime,
    #               sensorLib.getTotalAccel()/9.81,
    #               curAlt,
    #               "descending",
    #               "no",
    #               velocity)

    if curAlt <= DEPLOY_ALT and velocity < 0:
        print("Payload deployment conditions met")

        # Servos are locked at 150 degrees (this should already be set)
        # Servos unlock at 50 degrees
        # Servo 14 deploys at 300 feet
        servo1.write_angle(50)
        time.sleep(2)
        servo1.disable()

        payload_deployed = True
        print("Payload deployed")

    prevAlt = curAlt
    prevTime = curTime

    time.sleep(0.05)

# Land Detection
while True:
    altitude = (bmp.altitude - ground_alt)
    print("Altitude: ", altitude)
    #accel = sensorLib.getTotalAccel()/9.81

    #writeLogPacket(startTime, accel, altitude, "landing", "no", 0)

    if abs(altitude) < 3:
        print("Landing confirmed. Moving to soil collection...")
        # Harness (servo 15) deploy after landing 
        servo2.write_angle(50)
        time.sleep(2)
        servo2.disable()
        break

    time.sleep(0.1)




