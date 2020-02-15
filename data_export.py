import matplotlib.pyplot as plt
import csv
import pandas
import numpy
import datetime
from prettytable import PrettyTable

# Plots flight graph from data in 'filename' csv, saves to FlightGraphs folder
def flightgraph (filename, v0, tau_dot, r0):
	# Plotting data with MatPlotLib
	x = []
	y = []
	currentTime = datetime.datetime.now()

	# Get most recent csv
	df = pandas.read_csv(filename)

	# Get flight data and ideal calculations
	startTime = df["t"][0]
	x=df["t"] - startTime
	y=df["r_filt"]
	yIdeal = r0 * pow((1+((tau_dot*v0*x)/r0)), 1/tau_dot)

	# Plot both lines
	plt.plot(x,y, label='Flight Data')
	plt.plot(x,yIdeal, label='Ideal Flight with EF')
	plt.legend()
	plt.grid()
	plt.xlabel('Time (s)')
	plt.ylabel('Range (m)')
	plt.title('Range vs. Time')

	# Save file to FlightGraphs folder
	fileName = "{}{}".format("FlightGraphs/FlightGraph_", currentTime.strftime("%Y-%m-%d-%H:%M"))
	plt.savefig(fileName)

# Records flight data in .txt and .csv in folder and a copy as MostRecentData.csv in root
def writedata(start_height, stop_height, start_point, v0, tau_dot, r, t, r_filt, v, tau, v_need, a_need, cmnd, marker):
	currentTime = datetime.datetime.now()

	# Readable Table
	filename_readable = "{}{}{}".format("FlightData/Readable_", currentTime.strftime("%Y-%m-%d-%H:%M"),".txt")
	f = open(filename_readable, "w")
	x = PrettyTable()
	x.field_names = ["r","t","r_filt","v","tau","v_need","a_need","cmnd,marker"]
	header = "start_height = {}\nstop_height = {}\nstart_point = {}\nv0 = {}\ntau_dot = {}\nlen(r) = {}\n\n".format(start_height, stop_height, start_point, v0, tau_dot, len(r))
	f.write(header)
	for index in range(0, len(r)):
		x.add_row([r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index]])
	f.write(x.get_string())
	f.close()

	# CSV Data File and MostRecentData.csv
	filename = "{}{}{}".format("FlightData/Raw_", currentTime.strftime("%Y-%m-%d-%H:%M"),".csv")
	g = open(filename, "w")
	h = open("MostRecentData.csv", "w")
	g.write("r,t,r_filt,v,tau,v_need,a_need,cmnd,marker\n")
	h.write("r,t,r_filt,v,tau,v_need,a_need,cmnd,marker\n")
	for index in range(0, len(r)):
		newLine = "{},{},{},{},{},{},{},{},{}\n".format(r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index],marker[index])
		g.write(newLine)
		h.write(newLine)
	g.close()

def printRecentGraph(filename, offset):
	# Plotting data with MatPlotLib
	h = 1
	tau_dot = 0.5
	rawdata = []
	rawdata=numpy.loadtxt(filename, dtype='double', delimiter=',')
	samples = int(rawdata[0])

	# Get flight data and ideal calculations
	t = rawdata[samples+h:2*samples+h-1]
	start = t[0]
	t = [x  - start for x in t]
	r_filt = rawdata[2*samples+h:3*samples+h-1]
	v = rawdata[3*samples+h:4*samples+h-1]
	r0 = r_filt[offset]
	v0 = v[offset]
	tIdeal = numpy.linspace(0,-r0/(v0*tau_dot), samples)
	yIdeal = r0 * pow((1+((tau_dot*v0*tIdeal)/r0)), 1/tau_dot)
	# Plot both lines
	begin = t[offset]
	tIdeal = [x  + begin for x in tIdeal]
	plt.plot(t, r_filt, label='Flight Data')
	plt.plot(tIdeal, yIdeal, '-', label='Ideal Flight with EF')
	plt.legend()
	plt.grid()
	plt.xlabel('Time (s)')
	plt.ylabel('Range (m)')
	plt.title('Range vs. Time')
	plt.xlim(left = 0)
	plt.ylim(bottom = 0)
	plt.show()

	i = 0
	mseSum = 0
	n = len(r_filt)
	for i in range (0,n):
		dif = yIdeal[i] - r_filt[i]
		mseSum = mseSum + dif**2
	MSE = mseSum/n
	# Save file to FlightGraphs folder
	# fileName = "{}{}".format("FlightGraphs/FlightGraph_", currentTime.strftime("%Y-%m-%d-%H:%M"))
	# plt.savefig(fileName)
	print "MSE = " + str(MSE)



printRecentGraph("Justin.txt", 13)