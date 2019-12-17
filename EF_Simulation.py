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

time = np.linspace(0, 10, 500)
yIdeal = r0 * pow((1+((tau_dot*v0*time)/r0)), 1/tau_dot)
y2 = yIdeal * pow((1+((tau_dot*v0*time)/yIdeal)), 1/tau_dot)

#Plot simulation
plt.plot(time,yIdeal, label='Ideal Flight with EF')
plt.plot(time,y2, label='Y2')
plt.legend()
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Range (m)')
plt.title('Range vs. Time')

# Save file to FlightGraphs folder
fileName = "Simulations/ForwardLookingEF"
plt.savefig(fileName)