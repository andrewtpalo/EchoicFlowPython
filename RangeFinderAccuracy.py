import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress

ActualDist = []
SensorDist = []
with open('RangefinderTestingData.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    next(csv_reader, None)
    for row in csv_reader:
        ActualDist.append(float(row[0]))
        SensorDist.append(float(row[1]))

coefficient = np.polyfit(ActualDist, SensorDist, 1)
slope, intercept, r, p, std_err = linregress(ActualDist, SensorDist)
print(slope)
print(intercept)
regression = np.poly1d(coefficient)
plt.plot(ActualDist, SensorDist, 'bx', ActualDist, regression(ActualDist))
plt.xlim(0, 1100)
plt.ylim(0, 1100)
plt.grid()
plt.xlabel('Actual Distance (mm)')
plt.ylabel('Measured Distance (mm)')
plt.title('Sensor Accuracy')
plt.savefig('SensorAccuracy.png')