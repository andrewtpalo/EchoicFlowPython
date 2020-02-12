import time
import math
import matplotlib.pyplot as plt
import csv
import pandas
import data_export
import numpy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from random import randint


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
stage = "ef"
timer = "unset"

# //Parameters//

filename_readable = "recentKFTestReadable.txt"
filename = "recentKFTestBuff.csv"
start_height = 2.5
stop_height = 0.5
start_point = 0
v0 = -1.0
tau_dot = 0.5




# kf = KalmanFilter(dim_x=2, dim_z=1)
# dt = 1.0/15.0
# kf.x = numpy.array([[1.6],
# 					[-0.4]])
# kf.F = numpy.array([[1.,dt],
# 				[0.,1.]])

# kf.H = numpy.array([[1.,0.]])

# kf.P *= 10

# kf.R = 0.0001

# kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.13)
# // Velocity Equation //

# // C = 2.602*(Sqrt(0.712-V)-0.846)

# // Marker notation //
# //0 -> no EF, trying to reach constant velocity
# //1 -> starting EF from filtered data

# //Loop


# //Functions




		




def ComputeVelocity(r1,r2,t1,t2, prev):
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

f = open("ContinuousDataKFTest.txt", "w")
f.write("stage\tr\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
count = 0

current_range = start_height- stop_height
loop = True
startClock = time.time()
current_vel = v0
v.append(v0)
r.append(current_range)
t.append(0)
r_filt.append(current_range)
tau.append(0)
a_need.append(0)
v_need.append(0)
cmnd.append(0)
marker.append(0)
while loop:
	print current_range
	time.sleep(1.0/15.0)
	current_range = current_range  + (1.0/15.0)*current_vel + numpy.random.normal(0,0.005)
	current_time = time.time() - startClock
	r.append(current_range)
	t.append(current_time)

	#//what is the current sample?
	cur = len(r)-1
	prev = len(r) - 2

	#kf.predict()
	#kf.update(current_range)
	# current_filt = kf.x[0][0]
	# r_filt.append(current_filt)
	r_filt.append(current_range)


	#//compute current velocity
	# v.append(kf.x[1][0])
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur], prev))
	
	#//compute current tau
	tau.append(ComputeTau(r_filt[cur],v[cur]))

	#//compute needed acceleration
	a_need.append(v[cur]*(1-tau_dot)/tau[cur])

	#//compute needed velocity
	v_need.append(v[cur]+a_need[cur]*1.0/15.0)
	
	#//set speed to needed velocity
	cmnd.append(v_need[cur])
	current_vel = v_need[cur]
	#//save the marker
	marker.append(1)

	#//check if desitnation is reached
	if(current_range <= 0.01):
		loop = False
	WriteContinuously(f, count)
	count = count+1
		



	
f.close()



just = open("JustinKFTest.txt", "w")
just.write("{},".format(len(r)))
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
just.write("0")
just.close()

data_export.printRecentGraph("JustinKFTest.txt",0)