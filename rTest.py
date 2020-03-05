import echoic_flow_lr_test
import echoic_flow_test
import echoic_flow_kf_test
import numpy



n = 100

rValues = [50, 55, 60, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 100, 105, 110, 115]
for r in rValues:
    kfUnder10 = 0
    rmseKF = []
    bestKF= 10000
    for i in range(0,n):
        test = echoic_flow_kf_test.kfTest(0.031,0.0184,30, r)
        rmseKF.append(test)
        bestKF = min(bestKF, test)
        if test < .1:
            kfUnder10 = kfUnder10 + 1
    print "\n-------------\n"
    print "R Value: " + str(r)
    print "-------------"
    print "MEAN:"
    print "KF RMSE: " + str(numpy.mean(rmseKF))
    print "-------------"
    print "MEDIAN:"
    print "KF RMSE: " + str(numpy.median(rmseKF))
    print "-------------"
    print "BEST RMSE:"
    print "KF RMSE: " + str(bestKF)
    print "-------------"
    print "STD DEVIATION:"
    print "KF RMSE: " + str(numpy.std(rmseKF))
    print "-------------"
    print "NUM UNDER 10:"
    print "KF RMSE: " + str(kfUnder10)