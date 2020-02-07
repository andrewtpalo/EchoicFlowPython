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
ndc = drone.NavDataCount
r = []
loop = True
while loop:
    while ndc == drone.NavDataCount:
		time.sleep(0.0001)
    r.append(drone.NavData["demo"][3]/100)
    if len(r) > 150:
        loop = False


measurementVar = numpy.var(r)

print measurementVar