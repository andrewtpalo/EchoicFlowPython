import matplotlib.pyplot as plt
import ps_drone
import time
import math


drone = ps_drone.Drone()
drone.startup()           # Connects to the drone and starts subprocesses
drone.reset()
while (drone.getBattery()[0] == -1):   
    time.sleep(0.1) # Wait until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True) 
drone.getNDpackage(["demo","altitude"]) 
altitude = []
timeTrack = []
drone.takeoff()

# client.takeoff(function() {
start = time.time()
command = 0.15
loop = True
drone.move(0,0,command,0)
while (drone.NavData["demo"][3]/100) < 2.0:
    time.sleep(0.067)
    altitude.append(drone.NavData["demo"][3]/100)
    timeTrack.append(time.time())
drone.stop()
time.sleep(2.0)
f = open("veldata.txt", "w")
drone.move(0,0,-command,0)
while (drone.NavData["demo"][3]/100) > 1.0:
    time.sleep(0.067)
    altitude.append(drone.NavData["demo"][3]/100)
    timeTrack.append(time.time())
    f.write("{},{},".format(altitude[-1], timeTrack[-1]))
drone.land()

sq = (command / 2.644 + 0.868)
expectedVel = 0.749 - math.pow(sq, 2)

print expectedVel
f.close()
timeTrack = [x - start for x in timeTrack]
plt.plot(timeTrack, altitude)
plt.xlabel('time')
plt.ylabel('altitude')
plt.show()

