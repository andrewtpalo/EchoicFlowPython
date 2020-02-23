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
def ComputeAcceleration(v1,v2,t1,t2):
	if (t2 == t1):
		return 0

	return ((v2-v1)/(t2-t1))


def ComputeVelocity(r1,r2,t1,t2):
	if (t2 == t1):
		return 0

	return ((r2-r1)/(t2-t1))

def ComputeTau(r,v):
	if(v==0.0):
		v = -0.001
	return r/v

def lrTest(v_error, r_error, buf_size, sizeOfBuffer):
	#initialization
	r = []
	t = []
	r_meas = []
	v = []
	tau = []
	a_need = []
	v_need = []
	a = []


	# //Parameters//

	r0 = 2.0
	v0 = -0.4
	tau_dot = 0.5





	# // Velocity Equation //

	# // C = 2.602*(Sqrt(0.712-V)-0.846)

	# // Marker notation //
	# //0 -> no EF, trying to reach constant velocity
	# //1 -> starting EF from filtered data

	# //Loop


	# //Functions


	dt = 1.0/15.0


			



	f = open("ContinuousDataKFTest.txt", "w")
	f.write("stage\tr\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
	count = 0

	r_filt = []
	r_filt.append(r0)
	v.append(v0)
	r_meas.append(r0 + numpy.random.normal(0.0,r_error))
	t.append(0)
	r.append(r0)
	tau.append(0)
	a_need.append(0)
	v_need.append(0)

	v_meas = []
	i = 1
	while i < buf_size + 1:

		v.append(v[i-1])
		t.append(t[i-1] + dt)
		a.append(ComputeAcceleration(v[i-1],v[i],t[i-1],t[i]))

		

		r.append(r[i-1] + v[i-1]*dt + 0.5*a[i-1]*(dt**2))

		r_meas.append(r[i] + numpy.random.normal(0.0,r_error))
		r_filt.append(r_meas[i])
		i = i + 1


		#//what is the current sample?

	i = buf_size+1
	duration = -r[buf_size]/(tau_dot * v[buf_size])
	samples = numpy.ceil(duration*15 + buf_size+1)
	while r_filt[i-1] > 0.01 and i < samples:

		# current_filt = kf.x[0][0]
		# r_filt.append(current_filt)

		v_meas.append(ComputeVelocity(r_filt[i-2], r_filt[i-1], t[i-2], t[i-1]))
		tau.append(ComputeTau(r_filt[i-1], v_meas[-1]))
		a_need.append(v_meas[-1]*(1-tau_dot)/tau[-1])
		v_need.append(v_meas[-1] + a_need[-1]*dt)
		#//compute current velocity
		# v.append(kf.x[1][0])
		v.append(v_need[-1] + numpy.random.normal(0.0,v_error))
		t.append(t[i-1] + dt)
		a.append(ComputeAcceleration(v[i-1],v[i],t[i-1],t[i]))
		
		#//compute current tau


		#//compute needed acceleration
		r.append(r[i-1] + v[i-1]*dt + 0.5*a[i-1]*(dt**2))
		r_meas.append(r[i] + numpy.random.normal(0,r_error))
		r_buffed = r[len(r) - sizeOfBuffer:i]
		t_buffed = t[len(r) - sizeOfBuffer:i]
		poly = numpy.polyfit(t_buffed, r_buffed, 2)
		curve = numpy.poly1d(poly)
		current_filt = curve(t[i])
		r_filt.append(current_filt)
		i = i + 1
		#//check if desitnation is reached




		
	f.close()



	just = open("JustinLRTest.txt", "w")
	just.write("{},".format(len(r)))
	for x in r_meas:
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
	just.write("0")
	just.close()

	return data_export.printRecentGraph("JustinLRTest.txt",buf_size)