import ps_drone
import time
import math
import matplotlib.pyplot as plt
import csv
import time
import RPi.GPIO as GPIO
import serial
import datetime

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, GPIO.LOW)
f = open("ForwardData.txt","w+")
ser = serial.Serial(port="/dev/ttyS0", baudrate=9600)        
currentTime = datetime.datetime.now()
range = []
flightTime = []
GPIO.output(4, GPIO.HIGH)
time.sleep(0.00002)
serialOutput = ser.read(6)
currRange = int(serialOutput[1:])
counter = 0
start = time.time()
range.append(currRange)
flightTime.append(0)
while(currRange/10 > 50):
    serialOutput = ser.read(6)
    currRange = int(serialOutput[1:])
    currTime = time.time() - start
    range.append(currRange)
    flightTime.append(currTime)
    counter = counter + 1
ser.close()
f.close()
GPIO.cleanup()
print str(time.time()-start)
plt.plot(flightTime, range, label='Flight Data')
plt.xlabel('Time (sec)')
plt.ylabel('Range (cm)')
fileName = "{}{}".format("FlightGraphs/FlightGraph_", currentTime.strftime("%Y-%m-%d-%H:%M"))
plt.savefig(fileName)