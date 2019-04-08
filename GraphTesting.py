import matplotlib.pyplot as plt
import csv
import pandas

# Plotting data with MatPlotLib
x = []
y = []

df = pandas.read_csv('recentNOBuff.csv')

x=df["t"]
y=df["r"]

plt.plot(x,y, label='Range over Time')
plt.xlabel('t')
plt.ylabel('r')
plt.title('Range over Time')
plt.savefig("RvTPlot")