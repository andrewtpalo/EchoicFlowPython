import ps_drone
import time
import math
import matplotlib.pyplot as plt
import csv
import time
#import RPi.GPIO as GPIO
#import serial
#import pandas
#import data_export

#The following program performs the following steps,
#	1 Pause 20 seconds to allow disconnection from monitor
#	2 Launch to static start height
#	3 Pause for 5 seconds
#	4 Approach object (flying backwards) to distance of 50cm at 10% speed
#	5 Pause 2 seconds
#	6 Land

#Initial sleep
time.sleep(20)

drone = ps_drone.Drone()
drone.startup()           # Connects to the drone and starts subprocesses
drone.reset()
while (drone.getBattery()[0] == -1):   
    time.sleep(0.1) # Wait until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True) 
drone.getNDpackage(["demo","altitude"]) 
#time.sleep(1.0)


#initialization
r = []
t = []
r_filt = []
v = []
tau = []
a_need = []
v_need = []
cmnd = []
marker = []
header = []
file_return = []
stage = "up"
timer = "unset"

# //Parameters//

filename_readable = "recentNOBuffReadable.txt"
filename = "recentNOBuff.csv"
start_height = 2.0
stop_height = 0.5
start_point = 12
v0 = 0.4
tau_dot = 0.75
buf_size = 1
order = 1

# // Velocity Equation //

# // C = 2.602*(Sqrt(0.712-V)-0.846)

# // Marker notation //
# //0 -> no EF, trying to reach constant velocity
# //1 -> starting EF from filtered data

# //Loop

drone.takeoff()
while (drone.NavData["demo"][0][2]):
	time.sleep(0.1)
# client.takeoff(function() {


# //Functions

def FlyToHeight(current_range,current_time):
	global stage
	objHeight = start_height-stop_height
	if(current_range < objHeight):
		drone.move(0,0,.5,0)
	else:
		stage = 'pause'

def Pause(current_range,current_time):
	global stage
	global timer
	#stop the drone and wait
	drone.stop()

	if (timer == 'unset'):
		global stage
		global timer
		time.sleep(2.5)
		stage = 'dec'
		timer = 'set'


def StartDecent(current_range,current_time):
	global stage
	#//save initial range and time
	global stage
	global r
	global t
	global r_filt
	global v
	global tau
	global a_need
	global v_need
	global cmnd
	global marker
	r.append(current_range)
	t.append(current_time)
	r_filt.append(current_range)
	v.append(0.0)
	tau.append(0.0)
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(GetMotorCommand(v0))
	marker.append(0.0)
	

	#//begin decent at initial velocity
	drone.move(0,0,-1*GetMotorCommand(v0),0)
	
	#//change stage to start controlling decent
	stage = 'buf'

def FillBuffer(current_range,current_time):
	global stage
	global r
	global t
	global r_filt
	global v
	global tau
	global a_need
	global v_need
	global cmnd
	global marker


# 	//save data
	r.append(current_range)
	t.append(current_time)
	
	#//what is the current sample?
	cur = len(r)-1
	prev = len(r)-2

	#//save the rest of the data
	r_filt.append(current_range)
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	tau.append(ComputeTau(r[cur],v[cur]))
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(-1*GetMotorCommand(v0))
	marker.append(0.0)

	#//if we have reached the starting sample...begin EF!
	if (len(r) == start_point):
		stage = 'ef'

def EchoicFlow(current_range,current_time):
	global stage
	global r
	global t
	global r_filt
	global v
	global tau
	global a_need
	global v_need
	global cmnd
	global marker
	#//save current range and time
	r.append(current_range)
	t.append(current_time)

	#//what is the current sample?
	cur = len(r)-1
	prev = len(r)-2

	#//filter the range data
	r_filt.append(current_range)

	#//compute current velocity
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	
	#//compute current tau
	tau.append(ComputeTau(r_filt[cur],v[cur]))

	#//compute needed acceleration
	a_need.append(v[cur]*(1-tau_dot)/tau[cur])

	#//compute needed velocity
	v_need.append(v[cur]+a_need[cur]*1.0/15.0)
	
	#//set speed to needed velocity
	cmnd.append(-1*GetMotorCommand(v_need[cur]))
	drone.move(0,0,cmnd[cur],0)

	#//save the marker
	marker.append(1)

	#//check if desitnation is reached
	if(current_range <= stop_height):
		stage = 'stop'


def LandSave(current_range,current_time):
	global stage
	global r
	global t
	global r_filt
	global v
	global tau
	global a_need
	global v_need
	global cmnd
	global marker
	global start_height
	global stop_height
	global start_point
	global v0, tau_dot, buf_size, order
	drone.land()
	#data_export.writedata(start_height, stop_height, start_point, v0, tau_dot, buf_size, order, r, t, r_filt, v, tau, v_need, a_need, cmnd, marker)

def GetMotorCommand(velocity):
	# sq = math.sqrt(0.749-velocity)
	# rnd = round(2.644*(sq-0.868)*1000)
	# command = rnd/1000

	command = velocity
	if (command <= 0):
		return 0.01
	elif (command >= 1):
		return 1
	else:
		return command

def ComputeVelocity(r1,r2,t1,t2):
	if (t2 == t1):
		return 0
	return ((r2-r1)/(t2-t1))

def ComputeTau(r,v):
	if(v==0.0):
		v = -0.001
	return r/v

def WriteContinuously(f, index):
	newLine = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(stage,r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index],marker[index])
	f.write(newLine)

f = open("ContinuousData.txt", "w")
#f.write("stage\tr\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
count = 0
ndc = drone.NavDataCount
loop = True
startClock = time.time()

#Wait until drone is ready for commands
while ndc == drone.NavDataCount:
	time.sleep(0.001)
current_range = (drone.NavData["demo"][3]/100)
current_time = time.time() - startClock

#Fly to starting height
while current_range < start_height:
    drone.move(0,0,.6,0)
    current_range = (drone.NavData["demo"][3]/100)
drone.stop()

#Pause 1 second
time.sleep(1)

#Decend at velocity
while(current_range > stop_height):
    drone.move(0,0,-.6,0)
    current_range = (drone.NavData["demo"][3]/100)
    newline = "{},{},".format(current_range, time.time() - startClock)
    f.write(newline)
    time.sleep(0.1)

drone.stop()

#Land
drone.land()

ndc = drone.NavDataCount
f.close()
