
import ps_drone
import time
import math
from KalmanFilter import KalmanFilter
import numpy


drone = ps_drone.Drone()
drone.startup()           # Connects to the drone and starts subprocesses
drone.reset()
while (drone.getBattery()[0] == -1):   
    time.sleep(0.1) # Wait until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True) 
drone.getNDpackage(["demo","altitude"]) 
#time.sleep(1.0)
dt = 1.0/15
kf = KalmanFilter(xDimension=2, zDimension=1)
kf.x = numpy.array([[2.0],
                     [-0.4]])       # initial state (location and velocity)

kf.F = numpy.array([[1.,dt],
                     [0.,1.]])    # state transition matrix

kf.H = numpy.array([[1.,0.]])    # Measurement function
kf.P *= 1000.                 # covariance matrix
kf.R = 5                      # state uncertainty
kf.Q = numpy.array([[.25*dt**4, .5*dt**3]],
                    [[ .5*dt**3,    dt**2]])

#initialization
r = []
t = []
r_filt = []
v = []
tau = []
a_need = []
v_need = []
cmnd = []
marker = []
header = []
file_return = []
stage = "up"
timer = "unset"

# //Parameters//

filename_readable = "recentNOBuffReadable.txt"
filename = "recentNOBuff.csv"
start_height = 2.0
stop_height = 0.4
start_point = 12
v0 = 0.4
tau_dot = 0.75
buf_size = 1
order = 1

# // Velocity Equation //

# // C = 2.602*(Sqrt(0.712-V)-0.846)

# // Marker notation //
# //0 -> no EF, trying to reach constant velocity
# //1 -> starting EF from filtered data

# //Loop

drone.takeoff()
while (drone.NavData["demo"][0][2]):
	time.sleep(0.1)
# client.takeoff(function() {


# 	// listen for the "keypress" event 
# 	keypress(process.stdin);

# 	process.stdin.on('keypress', function (ch, key) {

# 	  //land the drone
# 	  if (ch == 'l'){
# 		client.removeAllListeners('navdata')
# 		console.log('landing...');
# 		client.land();
# 	  }

# 	  if (key && key.ctrl && key.name == 'l') {
# 		Write();
# 	  }
# 	});

# 	process.stdin.setRawMode(true);
# 	process.stdin.resume();

# 	// start listening for altitude information


# //Functions

def FlyToHeight(current_range,current_time):
	print"up"
	global stage
	objHeight = start_height-stop_height
	newline = "curr range = {}	objective height = {}\n".format(current_range, objHeight)
	f.write(newline)
	if(current_range < objHeight):
		drone.move(0,0.05, 0.5, 0)
	else:
		stage = 'pause'

def Pause(current_range,current_time):
	print"Pause"
	#stop the drone and wait
	drone.stop()

	if (timer == 'unset'):
		global stage
		global timer
		time.sleep(2.5)
		stage = 'dec'
		timer = 'set'


def StartDecent(current_range,current_time):
	global stage
	#//save initial range and time
	r.append(current_range)
	t.append(current_time)
	r_filt.append(current_range)
	v.append(0.0)
	tau.append(0.0)
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(-1*GetMotorCommand(v0))
	marker.append(0.0)
	

	#//begin decent at initial velocity
	drone.move(0,0, -1*GetMotorCommand(v0), 0)
	
	#//change stage to start controlling decent
	stage = 'buf'

def FillBuffer(current_range,current_time):
	global stage
# 	//save data
	r.append(current_range)
	t.append(current_time)
	
	#//what is the current sample?
	cur = len(r)-1
	prev = len(r)-2

	z = numpy.array([[current_range],
                     [0]])  
	kf.predict()
	kf.update(z)

	#//save the rest of the data
	r_filt.append(kf.x)
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	tau.append(ComputeTau(r[cur],v[cur]))
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(-1*GetMotorCommand(v0))
	marker.append(0.0)

	#//if we have reached the starting sample...begin EF!
	if (len(r) == start_point):
		stage = 'ef'

def EchoicFlow(current_range,current_time):
	global stage
	#//save current range and time
	r.append(current_range)
	t.append(current_time)

	#//what is the current sample?
	cur = len(r)-1
	prev = len(r)-2
    z = numpy.array([[current_range],
                     [0]])  
	kf.predict()
	kf.update(z)

	#//filter the range data
	r_filt.append(kf.x)

	#//compute current velocity
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	
	#//compute current tau
	tau.append(ComputeTau(r_filt[cur],v[cur]))

	#//compute needed acceleration
	a_need.append(v[cur]*(1-tau_dot)/tau[cur])

	#//compute needed velocity
	v_need.append(v[cur]+a_need[cur]*1/15)
	
	#//set speed to needed velocity
	cmnd.append(-1*GetMotorCommand(v_need[cur]))
	drone.move(0,0,cmnd[cur],0)

	#//save the marker
	marker.append(1)

	#//check if desitnation is reached
	if(current_range <= 0):
		stage = 'stop'


def LandSave(current_range,current_time):
	drone.land()
	print"WRITING"
	Write()


def Write():
	f = open(filename_readable, "w")
	header = "start_height = {}\nstop_height = {}\nstart_point = {}\nv0 = {}\ntau_dot = {}\nbuf_size = {}\norder = {}\nr.length = {}\n\n".format(start_height, stop_height, start_point, v0, tau_dot, buf_size, order, len(r))
	f.Write(header)
	f.write("r\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
	for index in range(0, len(r)):
		newLine = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index],marker[index])
		f.write(newLine)
	f.close()

	g = open(filename, "w")
	g.write("r,t,r_filt,v,tau,v_need,a_need,cmnd,marker\n")
	for index in range(0, len(r)):
		newLine = "{},{},{},{},{},{},{},{},{}\n".format(r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index],marker[index])
		g.write(newLine)
	g.close()


def WriteContinuously(f, index):
	f.write("stage\tr\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
	newLine = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(stage,r[index],t[index],r_filt[index],v[index],tau[index],v_need[index],a_need[index],cmnd[index],marker[index])
	f.write(newLine)



# function Exit() {

# 	console.log('exiting...')
# 	process.exit();
	
# }

def GetMotorCommand(velocity):
    command = round(2.644*(math.sqrt(0.749-velocity)-0.868)*1000)/1000
    if (command <= 0):
            return 0.01
    elif (command >= 1):
            return 1
    else:
            return command

def ComputeVelocity(r1,r2,t1,t2):
	if (t2 == t1):
		return 0
	return ((r2-r1)/(t2-t1))

def ComputeTau(r,v):
	if(v==0.0):
		v = -0.001
	return r/v

print"{}".format(drone.NavData["demo"][0][2])
f = open("BigData.txt", "w")
header = "start_height = {}\nstop_height = {}\nstart_point = {}\nv0 = {}\ntau_dot = {}\nbuf_size = {}\norder = {}\nr.length = {}\n\n".format(start_height, stop_height, start_point, v0, tau_dot, buf_size, order, len(r))
f.Write(header)
count = 0
f.write("r\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
ndc = drone.NavDataCount
while not stage == "stop":
    while drone.NavDataCount == ndc: 
        time.sleep(0.001)
	current_range = (drone.NavData["demo"][3]/100)-stop_height
	current_time = time.time()
	if stage == 'up':
		FlyToHeight(current_range, current_time)
		print"{}".format(drone.NavData["demo"][0][2])
	elif stage == 'pause':
		Pause(current_range, current_time)
	elif stage == 'dec':
		StartDecent(current_range,current_time)
        WriteContinuously(f, count)
        count = count+1
	elif stage == 'buf':
		FillBuffer(current_range,current_time)
        WriteContinuously(f, count)
        count = count+1
	elif stage == 'ef':
		EchoicFlow(current_range,current_time)
        WriteContinuously(f, count)
        count = count+1
	elif stage == 'stop':
		print"stop"
		LandSave(current_range,current_time)
    ndc = drone.NavDataCount
f.close

# function FillBuffer(current_range,current_time) {
# 	//save data
# 	r.push(current_range);
# 	t.push(current_time);
	
# 	//what is the current sample?
# 	var cur = r.length-1;
# 	var prev = r.length-2;

# 	//if we have reached the starting point switch stages and start filtering
# 	if (r.length == start_point) {
# 		stage = 'ef';
# 		var buf_first = r.length-buf_size;
# 		var r_buffed = r.slice(buf_first,cur);
# 		var t_buffed = t.slice(buf_first,cur);
# 		//var poly = new Polyfit(t_buffed,r_buffed);
# 		//var curve = poly.getPolynomial(order);
# 		//var current_filt = curve(current_time);
# 		xk_buffer = kalman.filtering(current_range, xk, xk_buffer, xk_prev);
# 		xk_prev = math.matrix([[math.subset(xk_buffer, math.index(0,i))],[math.subset(xk_buffer, math.index(1,i))]]);
# 		r_filt.push(math.subset(xk_buffer, math.index(0,i)));
# 		v.push(math.subset(xk_buffer, math.index(1,i)));
# 		marker.push(1);
# 		i++;
# 	} else {
# 		r_filt.push(current_range);
# 		marker.push(0);
# 		v.push(ComputeVelocity(r[prev],r[cur],t[prev],t[cur]));
# 	}

# 	//save the rest of the data
	
	
# 	tau.push(ComputeTau(r[cur],v[cur]));
# 	a_need.push(0.0);
# 	v_need.push(0.0);
# 	cmnd.push(GetMotorCommand(v0));
	

# 	//if we have reached the starting sample...begin EF!
	
# }

# function EchoicFlow(current_range,current_time) {
# 	//save current range and time
# 	r.push(current_range);
# 	t.push(current_time);

# 	//what is the current sample?
# 	var cur = r.length-1;
# 	var prev = r.length-2;

# 	//filter the range data
# 	var buf_first = r.length-buf_size;
# 	var r_buffed = r.slice(buf_first,cur);
# 	var t_buffed = t.slice(buf_first,cur);
# 	// var poly = new Polyfit(t_buffed,r_buffed);
# 	// var curve = poly.getPolynomial(order);
# 	// var current_filt = curve(current_time);
# 	// r_filt.push(current_filt);

# 	xk_buffer = kalman.filtering(current_range, xk, xk_buffer, xk_prev);
# 	xk_prev = math.matrix([[math.subset(xk_buffer, math.index(0,i))],[math.subset(xk_buffer, math.index(1,i))]]);
# 	v.push(math.subset(xk_buffer, math.index(1,i)));

# 	//compute current tau
# 	tau.push(ComputeTau(r_filt[cur],v[cur]));

# 	//compute needed acceleration
# 	a_need.push(v[cur]*(1-tau_dot)/tau[cur]);

# 	//compute needed velocity
# 	v_need.push(v[cur]+a_need[cur]*1/15);
	
# 	//set speed to needed velocity
# 	cmnd.push(GetMotorCommand(v_need[cur]));
# 	client.down(cmnd[cur]);

# 	//save the marker
# 	marker.push(1);
# 	i++;
# 	//check if desitnation is reached
# 	if(current_range <= 0)
# 	{
# 		stage = 'stop';

# 	}
# }






