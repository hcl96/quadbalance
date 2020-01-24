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

## System Parameters
DT = 0.010
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
u_array = np.array([])
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
kf.R = np.array([[15000,0,0,0],[0,95000,0,0],[0,0,45000,0],[0,0,0,100000]]) # default is 1000, 10000

klqr1 = 40 # default 20
klqr2 = 80 # default 80
klqr = 0*np.array([[klqr1, klqr2, klqr1, klqr2],[-klqr1, -klqr2, klqr1, klqr2],[-klqr1, -klqr2, -klqr1, -klqr2],[klqr1, klqr2, -klqr1, -klqr2]])

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


def power_update(power, roll_ref, pitch_ref):
	if isData():
		
		u_input = sys.stdin.read(1)
		if power >= 1050 and power <= 2000:
			if u_input == "p":
				power += 20
			elif u_input == "l":
				power -= 20
			elif u_input == "w":
				pitch_ref = 20
			elif u_input == "s":
				pitch_ref = -20
			elif u_input == "a":
				roll_ref = 20
			elif u_input == "d":
				roll_ref = -20
			else:
				pass
		if power < 1050:
			power = 1050
		if power > 2000:
			power = 2000
	
	return power, roll_ref, pitch_ref

def motor_control(power, roll_ref, pitch_ref, klqr, kf_est):
	u_array = np.matmul(-klqr, kf_est)
	esc_drive(power-roll_ref-pitch_ref+int(u_array[0]), power+roll_ref-pitch_ref+int(u_array[1]), power+roll_ref+pitch_ref+int(u_array[2]), power-roll_ref+pitch_ref+ int(u_array[3]))
	# esc_drive(power, power, power, power)
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
	gpsp = GpsPoller() # create thread
	print("[Boot] GPS Configured")
	# Get initial time 
	initialDT=int(round(time.time()*1000))
	print("[Boot] Bootup Successful. Initializing Control Sequence ... ")
	time.sleep(1)
	# Print Main control panel
	print(" - - - - - - - - PyQuadBalance - - - - - - - - ")
	print(" |  Latitude  |  Longitude  |  Alt  | Power |  Phi  |  Theta  |  Psi  | Phidot | Thetadot | Psidot |    ")
	
	try:
		gpsp.start()
		while 1:
			try:
				ax,ay,az,gx,gy,gz = mpu6050_read() # read and convert mpu6050 data
				# mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
			except IOError:
				print(" |                                            XXXX Warning XXX IMU Disconnect XXX Warning XXX  |  ", end="\r")
				continue

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
			kf.predict()
			kf.update(z)
			phikfrec = np.append(phikfrec, kf.x[1]*(180.0/PI))
			thetakfrec = np.append(thetakfrec, kf.x[3]*(180.0/PI))

			# Update Actuators
			power, roll_ref, pitch_ref = power_update(power, roll_ref, pitch_ref)
			u_array = motor_control(power, roll_ref, pitch_ref, klqr, kf.x)
			if roll_ref > 0:
				roll_ref -= 0.5
			else:
				roll_ref += 0.5
			if pitch_ref > 0:
				pitch_ref -= 0.5
			else:
				pitch_ref += 0.5
			
			# Record keeping
			currentDT = int(round(time.time()*1000))
			dtrec = np.append(dtrec, currentDT-initialDT)
			
			phirec = np.append(phirec, phi_raw*(180.0/PI))
			philprec = np.append(philprec, philp*(180.0/PI))
			thetarec = np.append(thetarec, theta_raw*(180.0/PI))
			thetalprec = np.append(thetalprec, thetalp*(180.0/PI))
			phidotrec = np.append(phidotrec, gx)
			thetadotrec = np.append(thetadotrec, gy)
			# psirec = np.append(psirec, psi_raw*(180.0/PI))
			
			powerrec = np.append(powerrec, power)

			# ucontrolrec = np.append(ucontrolrec, u_array[0]-u_array[2])
			# psihprec = np.append(psihprec, psi*(180.0/PI))

			# print("{0:1.0f},{1:1.2f},{2:1.2f},{3:1.2f},{4:1.2f}".format(power, phidothp,float(kf.x[1]*(180.0/PI)),thetadothp,float(kf.x[3]*(180.0/PI))))
			if (printstat % 10) == 0:
				print(" | {0:2.7f} | {1:2.6f} | {2:3.1f} | {3:1.0f}  |  {4:3.1f}  |  {5:2.1f}  |  {6:2.1f}  |  {7:2.1f}  |  {8:2.1f}  |  {9:2.1f}  |    "\
					.format(gpsd.fix.latitude, gpsd.fix.longitude, gpsd.fix.altitude, power, float(kf.x[1])*(180.0/PI), float(kf.x[3])*(180.0/PI), psi_raw, gx, gy, gz), end="\r")
			printstat += 1
			# Sleep at some point
			time.sleep(DT-0.005)

	except KeyboardInterrupt:
		print(" | {0:2.7f} | {1:2.6f} | {2:3.1f} | {3:1.0f}  |  {4:3.1f}  |  {5:2.1f}  |  {6:2.1f}  |  {7:2.1f}  |  {8:2.1f}  |  {9:2.1f}  |    "\
				.format(gpsd.fix.latitude, gpsd.fix.longitude, gpsd.fix.altitude, power, float(kf.x[1])*(180.0/PI), float(kf.x[3])*(180.0/PI), psi_raw, gx, gy, gz), end="\r")
		print("Interrupt Received. Exit")

	finally:
		gpsp.running = False
		gpsp.join() # wait for the thread to finish what it's doing
		## Shut down motors and stop GPIO
		esc_drive(1000, 1000, 1000, 1000)
		pi.stop()
		## Write log file
		print("Writing Log File")
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
		## Revert console to normal function
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)