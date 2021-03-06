import time
import numpy
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from random import randint
import matplotlib.pyplot as plt
iterate = 0
actualVar = []
actualBias = []
kfActualVar = []
kfActualBias = []
movingAvgVar = []
movingAvgBias = []
vactualVar = []
vactualBias = []
vkfActualVar = []
vkfActualBias = []
vmovingAvgVar = []
vmovingAvgBias = []
while iterate < 100:
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
        asum = 0

        if index >= 3:
            i = 0
            while i < 3:
                asum += actual[index-i]
                i+=1
            movingAvg.append(asum/3)
            asum = 0
            i = 0
            while i < 3:
                asum += actualV[index-i]
                i+=1
            movingAvgV.append(asum/3)
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

    actualVar.append(numpy.var(actualError))
    actualBias.append(sum(actualError)/len(actualError))
    kfActualVar.append(numpy.var(kfActualError))
    kfActualBias.append(sum(kfActualError)/len(kfActualError))
    movingAvgVar.append(numpy.var(movingAvgError))
    movingAvgBias.append(sum(movingAvgError)/len(movingAvgError))




    # print actualVar
    # print actualBias
    # print kfActualVar
    # print kfActualBias
    # print movingAvgVar
    # print movingAvgBias


    # print "---------------"


    # plt.plot(t, actualError, label = "Measured Range")
    # plt.plot(t, kfActualError, label = "Kalman Filtered Range")
    # plt.plot(t, movingAvgError, label = "Moving Average Range")

    # plt.title('Simulation of a Drone\'s Range Error')
    # plt.ylabel('Range Error(m)')
    # plt.xlabel('Time (s)')
    # plt.legend()
    # plt.show()


    vactualVar.append(numpy.var(actualV))
    vactualBias.append(sum(actualV)/len(actualV))
    vkfActualVar.append(numpy.var(kfActualV))
    vkfActualBias.append(sum(kfActualV)/len(kfActualV))
    vmovingAvgVar.append(numpy.var(movingAvgV))
    vmovingAvgBias.append(sum(movingAvgV)/len(movingAvgV))

    iterate += 1


    # print vactualVar
    # print vactualBias
    # print vkfActualVar
    # print vkfActualBias
    # print vmovingAvgVar
    # print vmovingAvgBias

    # plt.plot(t, actualV, label = "Measured Velocity")
    # plt.plot(t, kfActualV, label = "Kalman Filtered Velocity")
    # plt.plot(t, movingAvgV, label = "Moving Average Velocity")

    # plt.title('Simulation of a Drone\'s Velocity Error')
    # plt.ylabel('Velocity Error(m/s)')
    # plt.xlabel('Time (s)')
    # plt.legend()
    # plt.show()
print sum(actualVar)/100
print sum(actualBias)/100
print sum(kfActualVar)/100
print sum(kfActualBias)/100
print sum(movingAvgVar)/100
print sum(movingAvgBias)/100
print sum(vactualVar)/100
print sum(vactualBias)/100
print sum(vkfActualVar)/100
print sum(vkfActualBias)/100
print sum(vmovingAvgVar)/100
print sum(vmovingAvgBias)/100