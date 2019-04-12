import Adafruit_BBIO.GPIO as GPIO
import time
import Adafruit_BBIO.UART as UART
import serial

#   3.3v Power      P9_03
#   GND             P9_01
#   Serial RX       P9_26
#   Pulse Range     P9_23
#   Data Output     data.txt

UART.setup("UART1")
GPIO.setup("P9_23", GPIO.OUT)
GPIO.output("P9_23", GPIO.LOW)
f = open("Data.txt","w+")
iterations = 50

ser = serial.Serial(port = "/dev/ttyO1", baudrate=9600)
ser.close()
ser.open()
for i in range(0, iterations):
    GPIO.output("P9_23", GPIO.HIGH)
    time.sleep(0.00002)
    GPIO.output("P9_23", GPIO.LOW)
    time.sleep(0.1)
    x = ser.read(6)
    #print x
    f.write(x)
ser.close()
f.close()
