import echoic_flow_lr_test
import echoic_flow_test
import echoic_flow_kf_test


rmseLR = 0
rmseKF = 0
rmseNoFilt = 0
n = 50
for i in range(0,n):
    rmseLR = rmseLR + echoic_flow_lr_test.lrTest(0.007, 0.00126, 30, 19)
    rmseKF = rmseKF + echoic_flow_kf_test.kfTest(0.007, 0.00126, 30)
    rmseNoFilt = rmseNoFilt + echoic_flow_test.efTest(0.007, 0.00126, 30)

print "LR RMSE: " + str(rmseLR / n)
print "KF RMSE: " + str(rmseKF / n)
print "No Filt RMSE: " + str(rmseNoFilt / n)