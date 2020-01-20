import matplotlib.pyplot as plt
import csv
import pandas
import datetime
import numpy as np
from prettytable import PrettyTable

#Starting conditions
tau_dot = .5
v0 = -.4
r0 = 1.6

#Equations
time1 = np.linspace(0, 5, 500)
time2 = np.linspace(5, 13, 500)
yIdeal = r0 * pow((1+((tau_dot*v0*(time2-5))/r0)), 1/tau_dot)
yDroneStationary = -0.064*time1*time1+0.32*time1+1.6

#Conversion point plotting: x=5
x = np.linspace(5,5,500)
y = np.linspace(0,3,500)


#Plot simulation
plt.plot(time1,yDroneStationary, 'b', label='Ideal Flight with EF')
plt.plot(time2,yIdeal, 'b')
plt.plot(x,y, 'k--')
#plt.plot(time,y2, label='Y2')
plt.legend()
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Range (m)')
plt.title('Range vs. Time')

# Save file to FlightGraphs folder
fileName = "Simulations/ForwardLookingEF"
plt.savefig(fileName)