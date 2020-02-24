import echoic_flow_lr_test
import echoic_flow_test
import echoic_flow_kf_test
import numpy


rmseLR = []
rmseKF = []
rmseNoFilt = []
bestLR = 10000
bestKF= 10000
bestNoFilt = 10000
n = 100

for i in range(0,n):
    test = echoic_flow_lr_test.lrTest(0.0126, 0.007, 20, 12)
    bestLR = min(bestLR, test)
    rmseLR.append(test)
    test = echoic_flow_kf_test.kfTest(0.0126, 0.007, 15)
    rmseKF.append(test)
    bestKF = min(bestKF, test)
    test = echoic_flow_test.efTest(0.0126, 0.007, 12)
    rmseNoFilt.append(test)
    bestNoFilt = min(bestNoFilt, test)


print "MEDIANS:"
print "LR RMSE: " + str(numpy.median(rmseLR))
print "KF RMSE: " + str(numpy.median(rmseKF))
print "No Filt RMSE: " + str(numpy.median(rmseNoFilt))

print "\n-------------\n"
print "BEST RMSE:"

print "LR RMSE: " + str(bestLR)
print "KF RMSE: " + str(bestKF)
print "No Filt RMSE: " + str(bestNoFilt)
