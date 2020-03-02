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
n = 1000
lrUnder10 = 0
kfUnder10 = 0
noFiltUnder10 = 0
for i in range(0,n):
    test = echoic_flow_lr_test.lrTest(0.05, 0.01, 30, 19)
    bestLR = min(bestLR, test)
    if test < .1:
        lrUnder10 = lrUnder10 + 1
    rmseLR.append(test)
    test = echoic_flow_kf_test.kfTest(0.05, 0.01,30)
    rmseKF.append(test)
    bestKF = min(bestKF, test)
    if test < .1:
        kfUnder10 = kfUnder10 + 1
    test = echoic_flow_test.efTest(0.05, 0.01, 30)
    rmseNoFilt.append(test)
    bestNoFilt = min(bestNoFilt, test)
    if test < .1:
        noFiltUnder10 = lrUnder10 + 1

print "\n-------------\n"
print "MEANS:"
print "LR RMSE: " + str(numpy.mean(rmseLR))
print "KF RMSE: " + str(numpy.mean(rmseKF))
print "No Filt RMSE: " + str(numpy.mean(rmseNoFilt))
print "\n-------------\n"
print "MEDIANS:"
print "LR RMSE: " + str(numpy.median(rmseLR))
print "KF RMSE: " + str(numpy.median(rmseKF))
print "No Filt RMSE: " + str(numpy.median(rmseNoFilt))
print "\n-------------\n"
print "BEST RMSE:"
print "LR RMSE: " + str(bestLR)
print "KF RMSE: " + str(bestKF)
print "No Filt RMSE: " + str(bestNoFilt)
print "\n-------------\n"
print "STD DEVIATIONS:"
print "LR RMSE: " + str(numpy.std(rmseLR))
print "KF RMSE: " + str(numpy.std(rmseKF))
print "No Filt RMSE: " + str(numpy.std(rmseNoFilt))
print "\n-------------\n"
print "NUM UNDER 10:"
print "LR RMSE: " + str(lrUnder10)
print "KF RMSE: " + str(kfUnder10)
print "No Filt RMSE: " + str(noFiltUnder10)