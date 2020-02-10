import time
import numpy
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from random import randint
import matplotlib.pyplot as plt

kf = KalmanFilter(dim_x=2, dim_z=1)
dt = 1.0/15.0
kf.x = numpy.array([[2.1],
                    [-0.4]])
kf.F = numpy.array([[1.,dt],
                [0.,1.]])

kf.H = numpy.array([[1.,0.]])

kf.P *= 1000

kf.R = 0.01

kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.15)


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
movingAvg.append(2.1)
movingAvgV.append(-.4)
expected.append(2.0)
expectedV.append(-.3)
actual.append(2.1)
kfActual.append(2.1)
actualV.append(-.4)
kfActualV.append(-.4)
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
    asum = 0

    if index >= 5:
        i = 0
        while i < 5:
            asum += actual[index-i]
            i+=1
        movingAvg.append(asum/5)
        asum = 0
        i = 0
        while i < 5:
            asum += actualV[index-i]
            i+=1
        movingAvgV.append(asum/5)
    else:
        movingAvg.append(actual[index])
        movingAvgV.append(actualV[index])
    index+=1
    
actualError = []
kfActualError = []
movingAvgError = []

i = 0
while i<index:
    actualError.append(actual[i] - expected[i])
    kfActualError.append(kfActual[i] - expected[i])
    movingAvgError.append(movingAvg[i] - expected[i])
    actualV[i] = actualV[i] + 0.3
    kfActualV[i] = kfActualV[i] + 0.3
    movingAvgV[i] = movingAvgV[i] + 0.3
    i+=1
actualVar =  numpy.var(actualError)
actualBias = sum(actualError)/len(actualError)
kfActualVar =  numpy.var(kfActualError)
kfActualBias = sum(kfActualError)/len(kfActualError)
movingAvgVar =  numpy.var(movingAvgError)
movingAvgBias = sum(movingAvgError)/len(movingAvgError)




print actualVar
print actualBias
print kfActualVar
print kfActualBias
print movingAvgVar
print movingAvgBias


print "---------------"


plt.plot(t, actualError, label = "Measured Range")
plt.plot(t, kfActualError, label = "Kalman Filtered Range")
plt.plot(t, movingAvgError, label = "Moving Average Range")

plt.title('Simulation of a Drone\'s Range Error')
plt.ylabel('Range Error(m)')
plt.xlabel('Time (s)')
plt.legend()
plt.show()


vactualVar =  numpy.var(actualV)
vactualBias = sum(actualV)/len(actualV)
vkfActualVar =  numpy.var(kfActualV)
vkfActualBias = sum(kfActualV)/len(kfActualV)
vmovingAvgVar =  numpy.var(movingAvgV)
vmovingAvgBias = sum(movingAvgV)/len(movingAvgV)




print vactualVar
print vactualBias
print vkfActualVar
print vkfActualBias
print vmovingAvgVar
print vmovingAvgBias

plt.plot(t, actualV, label = "Measured Velocity")
plt.plot(t, kfActualV, label = "Kalman Filtered Velocity")
plt.plot(t, movingAvgV, label = "Moving Average Velocity")

plt.title('Simulation of a Drone\'s Velocity Error')
plt.ylabel('Velocity Error(m/s)')
plt.xlabel('Time (s)')
plt.legend()
plt.show()
