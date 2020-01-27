const SerialPort = require('serialport');
const Gpio = require('onoff').Gpio;

//   3.3v Power      P1
//   GND             P6
//   Serial RX       P10
//   Pulse Range     P7
//   Data Output     data.txt

const pulse = new Gpio(4, 'out');
pulse.writeSync(0);

const port = new SerialPort('/dev/ttyS0', {
  baudRate: 9600
})
const parser = port.pipe(new Delimiter({ delimiter: '\n' }))

console.log("Start\n")
var i;
var serialOutput;
for (i = 0; i < 10; i++){
    pulse.writeSync(1);
    setTimeout(function() {}, 0.02);
    pulse.writeSync(0);
    setTimeout(function() {}, 100);
    //serialOutput = ser.read(6)
    const parser = port.pipe(new Delimiter({ delimiter: '\n' }))
    console.log("${parser}\n");
}
console.log("Stop/n");
