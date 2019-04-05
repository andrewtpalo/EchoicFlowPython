
import ps_drone
import time
import math
import KalmanFilter


drone = ps_drone.Drone()
drone.startup()           # Connects to the drone and starts subprocesses
drone.reset()
while (drone.getBattery()[0] == -1):   
    time.sleep(0.1) # Wait until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True) 
drone.getNDpackage(["demo","altitude"]) 
#time.sleep(1.0)


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
v0 = -0.4
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
		drone.moveForward(0.05)
		drone.moveUp(0.5)
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
	cmnd.append(GetMotorCommand(v0))
	marker.append(0.0)
	

	#//begin decent at initial velocity
	drone.moveDown(GetMotorCommand(v0))
	
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

	#//save the rest of the data
	r_filt.append(current_range)
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	tau.append(ComputeTau(r[cur],v[cur]))
	a_need.append(0.0)
	v_need.append(0.0)
	cmnd.append(GetMotorCommand(v0))
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

	#//filter the range data
	r_filt.append(current_range)

	#//compute current velocity
	v.append(ComputeVelocity(r_filt[prev],r_filt[cur],t[prev],t[cur]))
	
	#//compute current tau
	tau.append(ComputeTau(r_filt[cur],v[cur]))

	#//compute needed acceleration
	a_need.append(v[cur]*(1-tau_dot)/tau[cur])

	#//compute needed velocity
	v_need.append(v[cur]+a_need[cur]*1/15)
	
	#//set speed to needed velocity
	cmnd.append(GetMotorCommand(v_need[cur]))
	drone.moveDown(cmnd[cur])

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
f = open("BigData", "w")
header = "start_height = {}\nstop_height = {}\nstart_point = {}\nv0 = {}\ntau_dot = {}\nbuf_size = {}\norder = {}\nr.length = {}\n\n".format(start_height, stop_height, start_point, v0, tau_dot, buf_size, order, len(r))
f.Write(header)
count = 0
f.write("r\tt\tr_filt\tv\ttau\tv_need\ta_need\tcmnd\tmarker\n")
while not stage == "stop":
	WriteContinuously(f, count)
	current_range = (drone.NavData["demo"][3]/100)-stop_height
	current_time = time.time()
	if stage == 'up':
		FlyToHeight(current_range, current_time)
		print"{}".format(drone.NavData["demo"][0][2])
	elif stage == 'pause':
		Pause(current_range, current_time)
	elif stage == 'dec':
		StartDecent(current_range,current_time)
	elif stage == 'buf':
		FillBuffer(current_range,current_time)
	elif stage == 'ef':
		EchoicFlow(current_range,current_time)
	elif stage == 'stop':
		print"stop"
		LandSave(current_range,current_time)
	count = count+1
f.close
