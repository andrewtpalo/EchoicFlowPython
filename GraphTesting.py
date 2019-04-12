import matplotlib.pyplot as plt
import csv
import pandas

# Plotting data with MatPlotLib
x = []
y = []

df = pandas.read_csv('recentNOBuff.csv')

x=df["t"] - 1554753711.83
y=df["r"]
y2= 1.6 * pow((1.0 + 0.75*(1.6/-0.4)*x),(1.0/.75))
print(x)
print(y2)
plt.plot(x,y, label='Range over Time')
plt.plot(x,y2, label='EF')
plt.xlabel('t')
plt.ylabel('r')
plt.title('Range over Time')
plt.grid()
plt.legend()
plt.savefig("Test")