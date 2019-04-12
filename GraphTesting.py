import matplotlib.pyplot as plt
import csv
import pandas

# Plotting data with MatPlotLib
x = []
y = []

df = pandas.read_csv('recentNOBuff.csv')

v0 = 0.4
tau_dot = 0.75
r0 = 1.6

x=df["t"]-1554753711.83
y=df["r"]
x2 = x
y2 = r0 * pow((1+(tau_dot*r0/v0*x2)), 1/tau_dot)

plt.plot(x,y, label='Range over Time')
plt.plot(x2,y2, label='EF')
plt.legend()
plt.grid()
plt.xlabel('t')
plt.ylabel('r')
plt.title('Range over Time')
plt.savefig("Test")