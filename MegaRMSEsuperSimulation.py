import echoic_flow_lr_test
import echoic_flow_test
import echoic_flow_kf_test
import numpy
import matplotlib.pyplot as plt
import pandas

measurementError = numpy.linspace(0, 0.20, 20)
lr = []
kf = []
no = []
for j in range(20):
    rmseLR = []
    rmseKF = []
    rmseNoFilt = []
    bestLR = 10000
    bestKF= 10000
    bestNoFilt = 10000
    n = 100
    lrUnder10 = 0
    kfUnder10 = 0
    noFiltUnder10 = 0
    for i in range(0,n):
        test = echoic_flow_lr_test.lrTest(0.05, measurementError[j], 30, 19)
        bestLR = min(bestLR, test)
        if test < .1:
            lrUnder10 = lrUnder10 + 1
        rmseLR.append(test)
        test = echoic_flow_kf_test.kfTest(0.05,measurementError[j],30)
        rmseKF.append(test)
        bestKF = min(bestKF, test)
        if test < .1:
            kfUnder10 = kfUnder10 + 1
        test = echoic_flow_test.efTest(0.05, measurementError[j], 30)
        rmseNoFilt.append(test)
        bestNoFilt = min(bestNoFilt, test)
        if test < .1:
            noFiltUnder10 = noFiltUnder10 + 1

    lr.append(numpy.mean(rmseLR))
    kf.append(numpy.mean(rmseKF))
    no.append(numpy.mean(rmseNoFilt))

    print "{}".format(j)

#plt.plot(measurementError, no, label='No Filt')
plt.plot(measurementError, lr, label='Least Regression')
plt.plot(measurementError, kf, label='Kalman Filter')
plt.legend()
plt.grid()
plt.xlabel('Measurement Error (Std. in Meters)')
plt.ylabel('Mean RMSE')
plt.title('Measurement Error Simulations')
plt.xlim(left = 0)
plt.ylim(bottom = 0)
plt.legend
plt.show()
