import ps_drone
import time
import math
import matplotlib.pyplot as plt
import csv
import pandas
import data_export
import numpy


drone = ps_drone.Drone()
drone.startup()           # Connects to the drone and starts subprocesses
drone.reset()
while (drone.getBattery()[0] == -1):   
    time.sleep(0.1) # Wait until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True) 
drone.getNDpackage(["demo","altitude"]) 
r = []
t= []
loop = True


drone.takeoff()
while (drone.NavData["demo"][0][2]):
	time.sleep(0.1)
# client.takeoff(function() {
drone.stop()

start = time.time()
while loop:

    time.sleep(0.067)
    r.append(drone.NavData["demo"][3])
    t.append(time.time() - start)
    print drone.NavData["demo"][3]
    if len(r) > 250:
        loop = False

drone.land()
print 'hi'
mean = sum(r)/len(r)
measurementVar = numpy.var(r)

print mean
print measurementVar


plt.hist(r, bins=10)

plt.title('Measurement Noise')
plt.ylabel('Frequency')
plt.xlabel('Range (cm)')

plt.legend()
plt.show()