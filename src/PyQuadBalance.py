## Libraries
import math
import time
from time import *
import threading
import numpy as np
import pigpio
from gps import *
from mpu9250_i2c import *
from filterpy.kalman import KalmanFilter
import sys, select, tty, termios # for the input
import matplotlib.pyplot as plt

ARDUINO_SLAVE = 0x04

## System Parameters
DT = 0.025
PI = 3.14159
printstat = 0

dtrec = np.array([])
# Raw Euler Log
phirec = np.array([])
thetarec = np.array([])
psirec = np.array([])
phidotrec = np.array([])
thetadotrec = np.array([])
psidotrec = np.array([])
# Kalman Log
phikfrec = np.array([])
thetakfrec = np.array([])
psikfrec = np.array([])
# LP/HP 
phipre = 0;
philppre = 0;
philprec = np.array([])
psihprec = np.array([])
thetapre = 0;
thetalppre = 0;
thetalprec = np.array([])
thetalprec = np.array([])
psi_raw = 0

## ESC channel
pi = pigpio.pi()
ESC1 = 7
ESC2 = 24
ESC3 = 8
ESC4 = 20
power = 1050
powerrec = np.array([])
u_array = np.array([[0],[0],[0],[0]])
ref_array = np.array([[0],[0],[0],[0]])
integral_array = np.array([[0],[0],[0],[0]])
ucontrolrec = np.array([])
roll_ref = 0
pitch_ref = 0

gpsd = None

## Kalman Filter Initialization
kf = KalmanFilter(dim_x = 4, dim_z = 4)
kf.x = np.array([[0],[0],[0],[0]])
kf.F = np.array([[1,0,0,0],[DT,1,0,0],[0,0,1,0],[0,0,DT,1]])
kf.B = np.array([[0.0001,-0.0001,-0.0001,0.0001],[0.0000007,-0.0000007,-0.0000007,0.0000007],[0.0001,0.0001,-0.0001,-0.0001],[0.0000007,0.0000007,-0.0000007,-0.0000007]])
kf.H = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
kf.P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
kf.Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
kf.R = np.array([[200,0,0,0],[0,1000,0,0],[0,0,200,0],[0,0,0,1000]]) # default is 1000, 10000

klqr1 = 20 # default 20
klqr2 = 40 # default 80
klqr = np.array([[klqr1, klqr2, klqr1, klqr2],[-klqr1, -klqr2, klqr1, klqr2],[-klqr1, -klqr2, -klqr1, -klqr2],[klqr1, klqr2, -klqr1, -klqr2]])

## Functions & Classes
class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		global gpsd #bring it in scope
		gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
		self.current_value = None
		self.running = True #setting the thread running to true

	def run(self):
		global gpsd
		while gpsp.running:
			gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_phi_raw(y,z):
    phi_raw = math.atan2(y, z)
    return phi_raw

def get_theta_raw(x,y,z):
    theta_raw = math.atan2(x, dist(y,z))
    return theta_raw

def get_psi_raw(gz,gz0):
	psi_raw = gz0 + gz*DT*1.5
	return psi_raw

def high_pass(filteredpre, measurementpre, measurement):
	wc = 0.001 # default 0.001
	Khp1 = 1+wc/2
	Khp2 = wc/2-1
	value_filtered = (1/Khp1)*(measurement - measurementpre - Khp2*filteredpre)
	return value_filtered

def low_pass(filteredpre, measurement):
	klp = 0.03 # previous 0.05
	value_filtered = (1-klp)*filteredpre + klp*measurement
	return value_filtered

def esc_drive(u1, u2, u3, u4):
	pi.set_servo_pulsewidth(ESC1, u1)
	pi.set_servo_pulsewidth(ESC2, u2)
	pi.set_servo_pulsewidth(ESC3, u3)
	pi.set_servo_pulsewidth(ESC4, u4)


def reference_update_keyboard(power, roll_ref, pitch_ref):
	roll_ref = 0
	pitch_ref = 0
	if isData(): # if data coming in
		u_input = sys.stdin.read(1)
		if power >= 1050 and power <= 2000:
			if u_input == "p":
				power += 20
			elif u_input == "l":
				power -= 20
			elif u_input == "w":
				pitch_ref = -20
			elif u_input == "s":
				pitch_ref = 20
			elif u_input == "a":
				roll_ref = -20
			elif u_input == "d":
				roll_ref = 20
			else:
				pass
	if power < 1050:
		power = 1050
	if power > 2000:
		power = 2000
	
	return power, roll_ref, pitch_ref

def motor_control(power, roll_ref, pitch_ref, klqr, kf_est, integral_array):
	ref_array = np.array([[0], [0.0175*roll_ref], [0], [0.0175*pitch_ref]])
	u_array = np.matmul(klqr, ref_array-kf_est) + np.matmul(klqr, integral_array)
	u_array = u_array + np.array([[power],[power],[power],[power]])
	for i in range (1,4):
		if u_array[i] > 2000:
			u_array[i] = 2000
		elif u_array[i] < 1000:
			u_array[i] = 1000
		else:
			pass

	esc_drive(int(u_array[0]), int(u_array[1]), int(u_array[2]), int(u_array[3]))
	return u_array

## Main Loop
esc_drive(1000, 1000, 1000, 1000)
print("[Boot] Motor Initialized ...")
time.sleep(0.5)
# Configure Termios
old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
print("[Boot] User IO Interface Initialized ...")
time.sleep(0.1)


if __name__=="__main__":
	# Configure GPS
	# gpsp = GpsPoller() # create thread
	print("[Boot] GPS Configured")
	# Get initial time 
	initialDT=int(round(time.time()*1000))
	print("[Boot] Bootup Successful. Initializing Control Sequence ... ")
	time.sleep(1)
	# Print Main control panel
	print(" - - - - - - - - PyQuadBalance - - - - - - - - ")
	print(" | Power |  Phi  |  Theta  |  Psi  | Phidot | Thetadot | Psidot |    ")
	
	try:
		# gpsp.start()
		while 1:
			try:
				ax,ay,az,gx,gy,gz = mpu6050_read() # read and convert mpu6050 data
				power, roll_ref, pitch_ref = bus.read_i2c_block_data(ARDUINO_SLAVE,0,3)
				# mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
				error_counter = 0
			except IOError:
				if error_counter == 1:
					print(" |  XXXX Warning XXX IMU Disconnect XXX Warning XXX  |  ", end="\r")
				else:
					pass
				error_counter = 1;
				continue


			power = 1050+power*3.72
			roll_ref = (128-roll_ref)*0.1
			pitch_ref = (128-pitch_ref)*0.1
			# Calculate raw Euler angles
			phi_raw = get_phi_raw(ay, az)
			theta_raw = get_theta_raw(-ax, ay, az)
			psi_raw = get_psi_raw(gz, psi_raw)

			# Filter Phi to clean noise
			philp = low_pass(philppre, phi_raw)
			phipre = phi_raw
			philppre = philp
			# Filter Theta to clean noise
			thetalp = low_pass(thetalppre, theta_raw)
			thetapre = theta_raw
			thetalppre = thetalp
			
			# KalmanFilter
			z = np.array([[(gx-gxbias)*(PI/180.0)],[phi_raw],[(gy-gybias)*(PI/180.0)],[theta_raw]])
			kf.predict(u_array)
			kf.update(z)
			integral_array = integral_array + (ref_array - kf.x)*DT
			

			# Update Actuators
			#power, roll_ref, pitch_ref = reference_update(power, roll_ref, pitch_ref)
			u_array = motor_control(power, roll_ref, pitch_ref, klqr, kf.x, integral_array)
			
			# Record keeping
			currentDT = int(round(time.time()*1000))
			dtrec = np.append(dtrec, currentDT-initialDT)
			phirec = np.append(phirec, phi_raw*(180.0/PI))
			philprec = np.append(philprec, philp*(180.0/PI))
			thetarec = np.append(thetarec, theta_raw*(180.0/PI))
			thetalprec = np.append(thetalprec, thetalp*(180.0/PI))
			phidotrec = np.append(phidotrec, gx)
			thetadotrec = np.append(thetadotrec, gy)
			phikfrec = np.append(phikfrec, kf.x[1]*(180.0/PI))
			thetakfrec = np.append(thetakfrec, kf.x[3]*(180.0/PI))
			# psirec = np.append(psirec, psi_raw*(180.0/PI))
			
			powerrec = np.append(powerrec, power)

			# ucontrolrec = np.append(ucontrolrec, u_array[0]-u_array[2])
			# psihprec = np.append(psihprec, psi*(180.0/PI))

			# print("{0:1.0f},{1:1.2f},{2:1.2f},{3:1.2f},{4:1.2f}".format(power, phidothp,float(kf.x[1]*(180.0/PI)),thetadothp,float(kf.x[3]*(180.0/PI))))
			# print(" | {0:1.0f}  |  {1:3.1f}  |  {2:2.1f}  |  {3:2.1f}  |  {4:2.1f}  |  {5:2.1f}  |  {6:2.1f}  |    "\
			# 	.format(power, float(kf.x[1])*(180.0/PI), float(kf.x[3])*(180.0/PI), psi_raw, gx, gy, gz), end="\r")
			print(" | {0:1.0f}  |  {1:3.1f}  |  {2:2.1f}  |  {3:2.1f}  |  {4:2.1f}  |  {5:2.1f}  |  {6:2.1f}  |    "\
				.format(power, roll_ref, pitch_ref, psi_raw, gx, gy, gz), end="\r")
			# print(u_array)
			# Sleep at some point
			time.sleep(DT-0.005)

	except KeyboardInterrupt:
		print(" | {0:1.0f}  |  {1:3.1f}  |  {2:2.1f}  |  {3:2.1f}  |  {4:2.1f}  |  {5:2.1f}  |  {6:2.1f}  |    "\
			.format(power, float(kf.x[1])*(180.0/PI), float(kf.x[3])*(180.0/PI), psi_raw, gx, gy, gz), end="\r")
		print("Interrupt Received. Exit")

	finally:
		## Revert console to normal function
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
		# gpsp.running = False
		# gpsp.join() # wait for the thread to finish what it's doing
		## Shut down motors and stop GPIO
		esc_drive(1000, 1000, 1000, 1000)
		pi.stop()
		## Write log file
		print("\nWriting Log File")
		log = open("quadlog.txt", "w+")
		for i in range(0, np.shape(dtrec)[0]-1):
			log.write("{0:1.0f},{1:1.2f},{2:1.2f},{3:1.2f},{4:1.2f},{5:1.0f}\n".format(dtrec[i], phirec[i], thetarec[i], phidotrec[i], thetadotrec[i], powerrec[i]))
		log.close()
		## Print Results
		plt.subplot(211)
		plt.ylabel('Phi, Deg')
		plt.xlabel('Timestep')
		# plt.plot(phirec)
		plt.plot(philprec)
		plt.plot(phikfrec,'g--')
		# plt.plot(gxrec)
		# plt.plot(phidotrec)
		plt.subplot(212)
		plt.ylabel('Theta, Deg')
		plt.xlabel('Timestep')
		# plt.plot(thetarec)
		plt.plot(thetalprec)
		plt.plot(thetakfrec,'g--')
		# plt.plot(gyrec)
		# plt.plot(thetadotrec)
		plt.show()
		