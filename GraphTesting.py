import matplotlib.pyplot as plt
import csv
import pandas
import datetime

# Plotting data with MatPlotLib
x = []
y = []
currentTime = datetime.datetime.now()

# Get most recent csv
df = pandas.read_csv('recentNOBuff.csv')


# Initialize variables
v0 = -0.4
tau_dot = 0.5
r0 = 1.6

# Get ideal data
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