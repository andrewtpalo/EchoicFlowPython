import ps_drone
import time
import math
import matplotlib.pyplot as plt
import csv
import pandas
import data_export
import RPi.GPIO as GPIO
import serial
import datetime
import numpy

#Allow time to disconnect monitor
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

#Pi Pin Initialization
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, GPIO.LOW)
ser = serial.Serial(port="/dev/ttyS0", baudrate=9600)   
GPIO.output(4, GPIO.HIGH)
time.sleep(0.00002)


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
start_height = 0.75
start_dist = 3
stop_dist = 0.75
stop_height = 0.2
start_point = 30
v0 = -0.1
tau_dot = 0.5
buf_size = 19
order = 2

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
drone.stop()
time.sleep(0.1)
drone.stop()
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
		stage = 'mov'
		timer = 'set'

def MoveToRange(current_horiz_range,current_time):
	global stage
	if(current_horiz_range > start_dist+0.01):
	    drone.move(0,-.05,0,0)
	elif(current_horiz_range < start_dist-0.01):
            drone.move(0,.05,0,0)
	else:
	    stage = 'pause2'

def Pause2(current_range,current_time):
	global stage
	global timer
	#stop the drone and wait
	drone.stop()

	if (timer == 'unset'):
		global stage
		global timer
		time.sleep(3)
		stage = 'approach'
		timer = 'set'

def StartApproach(current_horiz_range,current_time):
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
	r.append(current_horiz_range)
	t.append(current_time)
	r_filt.append(current_horiz_range)
	v.append(0.0)
	tau.append(0.0)
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(-1*GetMotorCommand(v0))
	marker.append(0.0)
	

	#//begin approach at initial velocity
	drone.move(0,-1*GetMotorCommand(v0),0,0)
	
	#//change stage to start controlling decent
	stage = 'buf'

def FillBuffer(current_horiz_range,current_time):
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
	r.append(current_horiz_range)
	t.append(current_time)
	
	#//what is the current sample?
	cur = len(r)-1
	prev = len(r)-2
	if len(r) == start_point:
            stage = 'ef'
            buf_first = len(r) - buf_size
            r_buffed = r[buf_first:cur]
            t_buffed = t[buf_first:cur]
            poly = numpy.polyfit(t_buffed, r_buffed, order)
            curve = numpy.poly1d(poly)
            current_filt = curve(current_time)
            r_filt.append(current_filt)
            marker.append(1)
        else:
            r_filt.append(current_horiz_range)
            marker.append(0.0)

	#//save the rest of the data
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	tau.append(ComputeTau(r[cur],v[cur]))
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(-1*GetMotorCommand(v0))

def EchoicFlow(current_horiz_range,current_time):
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
	r.append(current_horiz_range)
	t.append(current_time)

	#//what is the current sample?
	cur = len(r)-1
	prev = len(r)-2

        buf_first = len(r) - buf_size
        r_buffed = r[buf_first:cur]
        t_buffed = t[buf_first:cur]
        poly = numpy.polyfit(t_buffed, r_buffed, order)
        curve = numpy.poly1d(poly)
        current_filt = curve(current_time)
	r_filt.append(current_filt)

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
	drone.move(0,cmnd[cur],0,0)

	#//save the marker
	marker.append(1)

	#//check if desitnation is reached
	if(current_horiz_range <= stop_dist):
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
	data_export.writedata(start_dist, stop_dist, start_point, v0, tau_dot, buf_size, order, r, t, r_filt, v, tau, v_need, a_need, cmnd, marker)

def GetMotorCommand(velocity):
    if(velocity>0.748):
        velocity = 0.748
    sq = math.sqrt(0.749-velocity)
    rnd = round(2.644*(sq-0.868)*1000)
    command = rnd/1000

    #command = velocity
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
f.write("stage\tr\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
count = 0
ndc = drone.NavDataCount
loop = True
startClock = time.time()
while loop:
    while ndc == drone.NavDataCount:
        time.sleep(0.001)
    current_range = (drone.NavData["demo"][3]/100)-stop_height
    current_time = time.time() - startClock
    serialOutput = ser.read(6)
    current_horiz_range = float(int(serialOutput[1:])/1000.0)
    if stage == 'up':
		FlyToHeight(current_range, current_time)
		print"up\n"
    elif stage == 'pause':
		drone.stop() 
		Pause(current_range, current_time)
		print"pause1\n"
    elif stage == 'mov':
		MoveToRange(current_horiz_range, current_time)
		timer = "unset"
		print"Move to range\n"
		print str(current_horiz_range) + "\n"
    elif stage == 'pause2':
		drone.stop() 
		Pause2(current_range, current_time)
		print"pause2\n"
    elif stage == 'approach':
		StartApproach(current_horiz_range,current_time)
		WriteContinuously(f, count)
		count = count+1
		print"approach\n"
    elif stage == 'buf':
		FillBuffer(current_horiz_range,current_time)
		WriteContinuously(f, count)
		count = count+1
		print"buf\n"
    elif stage == 'ef':
		EchoicFlow(current_horiz_range,current_time)
		WriteContinuously(f, count)
		count = count+1
		print"ef\n"
		print str(current_horiz_range) + "\n"
    elif stage == 'stop':
		LandSave(current_horiz_range,current_time)
		loop = False
		print"stop\n"
    ndc = drone.NavDataCount
f.close()

#Pi Pin Cleanup
ser.close()
GPIO.cleanup()

r0 = start_height - stop_height
v0flight = v0
data_export.flightgraph ("MostRecentData.csv", v[0], tau_dot, r[0])

just = open("Justin.txt", "w")
just.write("x,x,x,x,x,x,x,{},".format(len(r)))
for x in r:
	line = "{},".format(x)
	just.write(line)
for x in t:
	line = "{},".format(x)
	just.write(line)
for x in r_filt:
	line = "{},".format(x)
	just.write(line)
for x in v:
	line = "{},".format(x)
	just.write(line)
for x in tau:
	line = "{},".format(x)
	just.write(line)
for x in v_need:
	line = "{},".format(x)
	just.write(line)
for x in a_need:
	line = "{},".format(x)
	just.write(line)
for x in cmnd:
	line = "{},".format(x)
	just.write(line)
for x in marker:
	line = "{},".format(x)
	just.write(line)
just.close()
