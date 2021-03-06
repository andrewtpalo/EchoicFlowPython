import matplotlib.pyplot as plt
import ps_drone
import time


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
drone.setSpeed(.4)
start = time.time()
command = 0.2
loop = True
while (drone.NavData["demo"][3]/100) < 2.0:
    drone.moveUp(command)
    altitude.append(drone.NavData["demo"][3]/100)
    timeTrack.append(time.time())
drone.stop()
time.sleep(2.0)

while (drone.NavData["demo"][3]/100) > 1.0:
    drone.moveDown(command)
    altitude.append(drone.NavData["demo"][3]/100)
    timeTrack.append(time.time())
drone.land()

timeTrack = timeTrack - start
plt.plot(timeTrack, altitude)
plt.xlabel('time')
plt.ylabel('altitude')
plt.show(block=False)

