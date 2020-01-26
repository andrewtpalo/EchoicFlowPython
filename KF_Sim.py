import time
import numpy
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from random import randint
import matplotlib.pyplot as plt

kf = KalmanFilter(dim_x=2, dim_z=1)
dt = 1.0/15.0
kf.x = numpy.array([[2.0],
                    [-0.3]])
kf.F = numpy.array([[1.,dt],
                [0.,1.]])

kf.H = numpy.array([[1.,0.]])

kf.P *= 1000

kf.R = 5

kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.13)


expected = []
expectedV = []
actual = []
kfActual = []
actualV = []
kfActualV = []
t = []
movingAvg = []
movingAvgV = []
index = 1
movingAvg.append(2.0)
movingAvgV.append(-.3)
expected.append(2.0)
expectedV.append(-.3)
actual.append(2.0)
kfActual.append(2.0)
actualV.append(-.3)
kfActualV.append(-.3)
t.append(0)
start = time.time()
z = 2.0
while kf.x[0] > 0.4:
    z = z - 0.02 
    noiseZ = z + randint(-5, 5)*0.03
    kf.predict()
    kf.update(noiseZ)
    time.sleep(0.067)
    expected.append(z)
    actual.append(noiseZ)
    kfActual.append(kf.x[0])
    t.append(time.time() - start)
    expectedV.append(-.3)
    actualV.append((noiseZ - actual[index-1])/(t[index] - t[index-1]))
    kfActualV.append(kf.x[1])
    sum = 0

    if index >= 3:
        i = 0
        while i < 3:
            sum += actual[index-i]
            i+=1
        movingAvg.append(sum/3)
        sum = 0
        i = 0
        while i < 3:
            sum += actualV[index-i]
            i+=1
        movingAvgV.append(sum/3)
    else:
        movingAvg.append(actual[index])
        movingAvgV.append(actualV[index])
    index+=1
    
sum = 0
i = 0
while i<index:
    sum += (actual[i] - expected[i])
    i+=1
bias =  sum / index


print bias

plt.subplot(2,1,1)
plt.plot(t, expected, label = "Expected Range")
plt.plot(t, actual, label = "Measured Range")
plt.plot(t, kfActual, label = "Kalman Filtered Range")
plt.plot(t, movingAvg, label = "Moving Average Range")

plt.title('Simulation of a Drone\'s Range')
plt.ylabel('Position (m)')
plt.xlabel('Time (s)')
plt.legend()

plt.subplot(2,1,2)
plt.plot(t, expectedV, label = "Expected Velocity")
plt.plot(t, actualV, label = "Measured Velocity")
plt.plot(t, kfActualV, label = "Kalman Filtered Velocity")
plt.plot(t, movingAvgV, label = "Moving Average Velocity")

plt.title('Simulation of a Drone\'s Velocity')
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.legend()
plt.show()
