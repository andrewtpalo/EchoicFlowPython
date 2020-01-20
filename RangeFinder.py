import RPi.GPIO as GPIO
import serial
import matplotlib
import matplotlib.pyplot as plt
import time
 
#   3.3v Power      P1
#   GND             P6
#   Serial RX       P10
#   Pulse Range     P7
#   Data Output     data.txt

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, GPIO.LOW)
f = open("Data.txt","w+")
iterations = 10
x = []
y = []

ser = serial.Serial(port="/dev/ttyS0", baudrate=9600)
startTime = time.time()
print("Start")
for i in range(0, iterations):
    GPIO.output(4, GPIO.HIGH)
    time.sleep(0.00002)
    GPIO.output(4, GPIO.LOW)
    time.sleep(0.1)
    serialOutput = ser.read(6)
    y.append(int(serialOutput[1:]))
    #print x
    x.append(time.time() - startTime)
    f.write(str(int(serialOutput[1:]))+" ")
ser.close()
f.close()
GPIO.cleanup()
print("Done")


plt.plot(x,y, label='Range over Time')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Dist From Sensor (mm)')
plt.title('Range over Time')
plt.ylim([0, 500])
plt.savefig("Test")