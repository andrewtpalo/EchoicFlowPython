import matplotlib.pyplot as plt
import csv
import pandas
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
    x=df["t"]-1554753711.83
    y=df["r"]
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
def Write():
    currentTime = datetime.datetime.now()

    # Readable Table
	filename_readable = "{}{}{}".format("FlightData/Readable_", currentTime.strftime("%Y-%m-%d-%H:%M"),".txt")
    f = open(filename_readable, "w")
	x = PrettyTable()
	x.field_names = ["r","t","r_filt","v","tau","v_need","a_need","cmnd,marker"]
	header = "start_height = {}\nstop_height = {}\nstart_point = {}\nv0 = {}\ntau_dot = {}\nbuf_size = {}\norder = {}\nlen(r) = {}\n\n".format(start_height, stop_height, start_point, v0, tau_dot, buf_size, order, len(r))
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
    g.write("r,t,r_filt,v,tau,v_need,a_need,cmnd,marker\n")
	for index in range(0, len(r)):
		newLine = "{},{},{},{},{},{},{},{},{}\n".format(r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index],marker[index])
		g.write(newLine)
        h.write(newLine)
	g.close()
