
'''
AUTHOR: Aditya Patel and Jim Ramsay
DATE CREATED: 2018-03-01
LAST MODIFIED: 2018-04-12
PLATFORM: Raspberry Pi 3B, Raspbian Stretch Released 2017-11-29
PROJECT: EMG Human Machine Interface
ORGANIZATION: Bradley University, School of Electrical and Computer Engineering
FILENAME: myo_raw.py
DESCRIPTION:
	Main script that:
		- initializes/executes bluetooth protocol (not written by Aditya/Jim -- see note below)
		- starts reading emg data. 
		- detects gestures
		- commands two slave raspberry pi's to rotate servo motors
		- switches between displaying video feed from each of the slaves
		
	Gestures (right hand only, have not tested on left hand):
		rest -- do nothing, arm relaxed
		fist -- tight fist
		piwo -- palm in, wrist out (wave outward)
		piwi -- palm in, wrist in (wave inward)
	
	Master: 
		emgPi_3 -- pi@169.254.12.52 password is "ee00"
	
	Slaves:
		ssh commands recognize the defined names for the slaves using ssh_keys. Using the defined
		names and saved keys bypasses password requirements. 
		
		emgPi_1 -- pi@169.254.184.5  password is "ee00"
		emgPi_2 -- pi@169.254.13.230 password is "ee00"
			
NOTE: 
	Original by dzhu
		https://github.com/dzhu/myo-raw

	Edited by Fernando Cosentino
		http://www.fernandocosentino.net/pyoconnect
		
	Edited further by Aditya Patel and Jim Ramsay
		There are a lot of global variables used to function like constants. This is likely not good practice
		but had to be done to meet deadlines. 
'''

from __future__ import print_function
import enum
import re
import struct
import sys
import threading
import time
import string
import serial
from serial.tools.list_ports import comports
from common import *

''' Additional Imports '''
import os
import numpy as np
import csv
import datetime
from ringBuffer import ringBuffer
import displayControl as display
from calibrate import Calibrate
from guppy import hpy

''' 
	GLOBAL VARIABLES 
	note: a lot of these are meant to function like a "DEFINE" in C. They are never written to.
'''

''' ARRAYS '''
global emg_data 
emg_data = []

global duty
duty = [50, 50]																			# initial duty cycle for each motor

''' INTEGERS '''
global GETTINGCALDATA; global CALIBRATING; global SLEEP; global WAITING; global DISPLAYCONTROL; global MOTORCONTROL
GETTINGCALDATA = 0
CALIBRATING = 1
SLEEP = 2
WAITING = 3
DISPLAYCONTROL = 4
MOTORCONTROL = 5

global REST; global FIST; global PIWI; global PIWO
REST = 0
FIST = 1
PIWI = 2
PIWO = 3

global calMode
calMode = REST

global EMGPI_1; global EMGPI_2
EMGPI_1 = 0
EMGPI_2 = 1


global fistCalData;	global piwiCalData;	global piwoCalData;
fistCalData = []
piwiCalData = []
piwoCalData = []

global curPi
curPi = 0

t0 = time.time()
global t_endWaiting

gestureString = ["fist", "piwi", "piwo", ""]
modeString = ["", "", "SLEEP", "WAITING","DISPLAY CONTROL","MOTOR CONTROL"]

def multichr(ords):
	if sys.version_info[0] >= 3:
		return bytes(ords)
	else:
		return ''.join(map(chr, ords))

def multiord(b):
	if sys.version_info[0] >= 3:
		return list(b)
	else:
		return map(ord, b)

class Arm(enum.Enum):
	UNKNOWN = 0
	RIGHT = 1
	LEFT = 2

class XDirection(enum.Enum):
	UNKNOWN = 0
	X_TOWARD_WRIST = 1
	X_TOWARD_ELBOW = 2

class Pose(enum.Enum):
	RESTT = 0
	FIST = 1
	WAVE_IN = 2
	WAVE_OUT = 3
	FINGERS_SPREAD = 4
	THUMB_TO_PINKY = 5
	UNKNOWN = 255

class Packet(object):
	def __init__(self, ords):
		self.typ = ords[0]
		self.cls = ords[2]
		self.cmd = ords[3]
		self.payload = multichr(ords[4:])

	def __repr__(self):
		return 'Packet(%02X, %02X, %02X, [%s])' % \
			(self.typ, self.cls, self.cmd,
			 ' '.join('%02X' % b for b in multiord(self.payload)))

class BT(object):
	'''Implements the non-Myo-specific details of the Bluetooth protocol.'''
	def __init__(self, tty):
		self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
		self.buf = []
		self.lock = threading.Lock()
		self.handlers = []

	## internal data-handling methods
	def recv_packet(self, timeout=None):
		t0 = time.time()
		self.ser.timeout = None
		while timeout is None or time.time() < t0 + timeout:
			if timeout is not None: self.ser.timeout = t0 + timeout - time.time()
			c = self.ser.read()
			if not c: return None

			ret = self.proc_byte(ord(c))
			if ret:
				if ret.typ == 0x80:
					self.handle_event(ret)
				return ret

	def recv_packets(self, timeout=.5):
		res = []
		t0 = time.time()
		while time.time() < t0 + timeout:
			p = self.recv_packet(t0 + timeout - time.time())
			if not p: return res
			res.append(p)
		return res

	def proc_byte(self, c):
		if not self.buf:
			if c in [0x00, 0x80, 0x08, 0x88]:
				self.buf.append(c)
			return None
		elif len(self.buf) == 1:
			self.buf.append(c)
			self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
			return None
		else:
			self.buf.append(c)

		if self.packet_len and len(self.buf) == self.packet_len:
			p = Packet(self.buf)
			self.buf = []
			return p
		return None

	def handle_event(self, p):
		for h in self.handlers:
			h(p)

	def add_handler(self, h):
		self.handlers.append(h)

	def remove_handler(self, h):
		try: self.handlers.remove(h)
		except ValueError: pass

	def wait_event(self, cls, cmd):
		res = [None]
		def h(p):
			if p.cls == cls and p.cmd == cmd:
				res[0] = p
		self.add_handler(h)
		while res[0] is None:
			self.recv_packet()
		self.remove_handler(h)
		return res[0]

	## specific BLE commands
	def connect(self, addr):
		return self.send_command(6, 3, pack('6sBHHHH', multichr(addr), 0, 6, 6, 64, 0))

	def get_connections(self):
		return self.send_command(0, 6)

	def discover(self):
		return self.send_command(6, 2, b'\x01')

	def end_scan(self):
		return self.send_command(6, 4)

	def disconnect(self, h):
		return self.send_command(3, 0, pack('B', h))

	def read_attr(self, con, attr):
		self.send_command(4, 4, pack('BH', con, attr))
		return self.wait_event(4, 5)

	def write_attr(self, con, attr, val):
		self.send_command(4, 5, pack('BHB', con, attr, len(val)) + val)
		return self.wait_event(4, 1)

	def send_command(self, cls, cmd, payload=b'', wait_resp=True):
		s = pack('4B', 0, len(payload), cls, cmd) + payload
		self.ser.write(s)

		while True:
			p = self.recv_packet()

			## no timeout, so p won't be None
			if p.typ == 0: return p

			## not a response: must be an event
			self.handle_event(p)

class MyoRaw(object):
	'''Implements the Myo-specific communication protocol.'''

	def __init__(self, tty=None):
		if tty is None:
			tty = self.detect_tty()
		if tty is None:
			raise ValueError('Myo dongle not found!')

		self.bt = BT(tty)
		self.conn = None
		self.emg_handlers = []
		self.imu_handlers = []
		self.arm_handlers = []
		self.pose_handlers = []

	def detect_tty(self):
		for p in comports():
			if re.search(r'PID=2458:0*1', p[2]):
				print('using device:', p[0])
				return p[0]

		return None

	def run(self, timeout=None):
		self.bt.recv_packet(timeout)

	def connect(self):
		## stop everything from before
		self.bt.end_scan()
		self.bt.disconnect(0)
		self.bt.disconnect(1)
		self.bt.disconnect(2)


		## start scanning
		print('scanning for bluetooth devices...')
		self.bt.discover()
		while True:
			p = self.bt.recv_packet()
			print('scan response:', p)

			if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
				addr = list(multiord(p.payload[2:8]))
				break
		self.bt.end_scan()

		## connect and wait for status event
		conn_pkt = self.bt.connect(addr)
		self.conn = multiord(conn_pkt.payload)[-1]
		self.bt.wait_event(3, 0)

		## get firmware version
		fw = self.read_attr(0x17)
		_, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
		print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

		self.old = (v0 == 0)

		if self.old: # if the firmware is 0.x.xxxx.x
			## don't know what these do; Myo Connect sends them, though we get data
			## fine without them
			self.write_attr(0x19, b'\x01\x02\x00\x00')
			self.write_attr(0x2f, b'\x01\x00')
			self.write_attr(0x2c, b'\x01\x00')
			self.write_attr(0x32, b'\x01\x00')
			self.write_attr(0x35, b'\x01\x00')

			## enable EMG data
			self.write_attr(0x28, b'\x01\x00')
			## enable IMU data
			self.write_attr(0x1d, b'\x01\x00')

			## Sampling rate of the underlying EMG sensor, capped to 1000. If it's
			## less than 1000, emg_hz is correct. If it is greater, the actual
			## framerate starts dropping inversely. Also, if this is much less than
			## 1000, EMG data becomes slower to respond to changes. In conclusion,
			## 1000 is probably a good value.
			C = 1000
			emg_hz = 50
			## strength of low-pass filtering of EMG data
			emg_smooth = 100

			imu_hz = 50

			## send sensor parameters, or we don't get any data
			self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C // emg_hz, imu_hz, 0, 0))

		else: #normal operation
			name = self.read_attr(0x03)
			print('device name: %s' % name.payload)

			## enable IMU data
			self.write_attr(0x1d, b'\x01\x00')
			## enable vibrations
			self.write_attr(0x24, b'\x02\x00')
			# Failed attempt to disable vibrations:
			# self.write_attr(0x24, b'\x00\x00')

			# self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
			self.start_raw()

		## add data handlers
		def handle_data(p):
			if (p.cls, p.cmd) != (4, 5): return
			c, attr, typ = unpack('BHB', p.payload[:4]) # unpack unsigned char, unsigned short, unsigned char
			pay = p.payload[5:]
			if attr == 0x27:
				vals = unpack('8HB', pay) 	# unpack 8 unsigned shorts, and one unsigned char https://docs.python.org/2/library/struct.html
											## not entirely sure what the last byte is, but it's a bitmask that
											## seems to indicate which sensors think they're being moved around or
											## something
				emg = vals[:8]
				moving = vals[8]
				self.on_emg(emg, moving)
			elif attr == 0x1c:
				vals = unpack('10h', pay)
				quat = vals[:4]
				acc = vals[4:7]
				gyro = vals[7:10]
				self.on_imu(quat, acc, gyro)
			elif attr == 0x23:
				typ, val, xdir, _,_,_ = unpack('6B', pay)

				if typ == 1: # on arm
					self.on_arm(Arm(val), XDirection(xdir))
					print("on arm")
				elif typ == 2: # removed from arm
					self.on_arm(Arm.UNKNOWN, XDirection.UNKNOWN)
					print("NOT on arm")
				elif typ == 3: # pose
					self.on_pose(Pose(val))
			else:
				print('data with unknown attr: %02X %s' % (attr, p))

		self.bt.add_handler(handle_data)

	def write_attr(self, attr, val):
		if self.conn is not None:
			self.bt.write_attr(self.conn, attr, val)

	def read_attr(self, attr):
		if self.conn is not None:
			return self.bt.read_attr(self.conn, attr)
		return None

	def disconnect(self):
		if self.conn is not None:
			self.bt.disconnect(self.conn)

	def start_raw(self):
		'''Sending this sequence for v1.0 firmware seems to enable both raw data and
		pose notifications.
		'''

		self.write_attr(0x28, b'\x01\x00')
		#self.write_attr(0x19, b'\x01\x03\x01\x01\x00')
		self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

	def mc_start_collection(self):
		'''Myo Connect sends this sequence (or a reordering) when starting data
		collection for v1.0 firmware; this enables raw data but disables arm and
		pose notifications.
		'''

		self.write_attr(0x28, b'\x01\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x24, b'\x02\x00')
		self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
		self.write_attr(0x28, b'\x01\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x19, b'\x09\x01\x01\x00\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x19, b'\x01\x03\x00\x01\x00')
		self.write_attr(0x28, b'\x01\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x19, b'\x01\x03\x01\x01\x00')

	def mc_end_collection(self):
		'''Myo Connect sends this sequence (or a reordering) when ending data collection
		for v1.0 firmware; this reenables arm and pose notifications, but
		doesn't disable raw data.
		'''

		self.write_attr(0x28, b'\x01\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x24, b'\x02\x00')
		self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
		self.write_attr(0x19, b'\x09\x01\x00\x00\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x24, b'\x02\x00')
		self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
		self.write_attr(0x28, b'\x01\x00')
		self.write_attr(0x1d, b'\x01\x00')
		self.write_attr(0x24, b'\x02\x00')
		self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

	def vibrate(self, length):
		if length in xrange(1, 4):
			## first byte tells it to vibrate; purpose of second byte is unknown
			self.write_attr(0x19, pack('3B', 3, 1, length))


	def add_emg_handler(self, h):
		self.emg_handlers.append(h)

	def add_imu_handler(self, h):
		self.imu_handlers.append(h)

	def add_pose_handler(self, h):
		self.pose_handlers.append(h)

	def add_arm_handler(self, h):
		self.arm_handlers.append(h)


	def on_emg(self, emg, moving):
		for h in self.emg_handlers:
			h(emg, moving)

	def on_imu(self, quat, acc, gyro):
		for h in self.imu_handlers:
			h(quat, acc, gyro)

	def on_pose(self, p):
		for h in self.pose_handlers:
			h(p)

	def on_arm(self, arm, xdir):
		for h in self.arm_handlers:
			h(arm, xdir)

	
def controlLogic(mode, gesture, confidence):
	global SLEEP; global WAITING; global DISPLAYCONTROL; global MOTORCONTROL;
	global REST; global FIST; global PIWI; global PIWO
	global duty; global curPi; global t_endWaiting; global t_30_SLEEP 
	
	if ( mode == SLEEP ):
	
		if ( gesture == FIST ):
			mode = WAITING
			t_endWaiting = time.time() + 1													# Reset the sleep timer once you leave SLEEP
			print("SWITCHING MODE: WAITING\t\t\t\tConfidence Level: ", confidence)
			t_30_SLEEP = time.time() + 30 
	
	if ( mode == WAITING ):
		if ( time.time() >= t_30_SLEEP ):
		
			mode = SLEEP
			print("SWITCHING MODE: SLEEP")
			
		else:
		
			# print("MODE = WAITING")
			if ( time.time() > t_endWaiting ):
				if ( gesture == FIST ):
				
					mode = SLEEP
					print("SWITCHING MODE: SLEEP\t\t\t\tConfidence Level: ",confidence)
					
				elif ( gesture == PIWI ):
					
					mode = DISPLAYCONTROL
					print("SWITCHING MODE: DISPLAYCONTROL\t\t\t\tConfidence Level: ",confidence)
					t_endWaiting = time.time() + 1	
					t_30_SLEEP = time.time() + 30
						
				elif ( gesture == PIWO ):
				
					mode = MOTORCONTROL	
					print("SWITCHING MODE: MOTORCONTROL\t\t\tConfidence Level: ",confidence)
					t_endWaiting = time.time() + 1	
					t_30_SLEEP = time.time() + 30
			
	if ( mode == DISPLAYCONTROL ):
		
		if ( time.time() >= t_30_SLEEP ):
		
			mode = SLEEP
			print("SWITCHING MODE: SLEEP")
			
		else:
		
			if ( time.time() > t_endWaiting ):
				if ( gesture == FIST ):
				
					mode = WAITING
					print("SWITCHING MODE: WAITING\t\t\t\tConfidence Level: ",confidence)
					t_endWaiting = time.time() + 1
					t_30_SLEEP = time.time() + 30
					
				elif ( ( curPi == 0 ) and ( gesture == PIWI ) ):

					curPi = display.switchDisplay()
					print("Switching to Camera 2")
					t_endWaiting = time.time() + 1
					t_30_SLEEP = time.time() + 30
				
				elif ( ( curPi == 1 ) and ( gesture == PIWO ) ):
					
					curPi = display.switchDisplay()
					print("Switching to Camera 1")
					t_endWaiting = time.time() + 1
					t_30_SLEEP = time.time() + 30
			
	if ( mode == MOTORCONTROL ):
		
		if ( time.time() >= t_30_SLEEP ):
		
			mode = SLEEP
			print("SWITCHING MODE: SLEEP")
			
		else:
		
			if ( time.time() > t_endWaiting ):
				''' Select which slave to control '''
				if ( curPi == 0 ):
					
					curPi_name = "emgPi_1"
					currentMotor = 0
					
				elif ( curPi == 1 ):
					
					curPi_name = "emgPi_2"
					currentMotor = 1
				
				''' Check Gesture '''
				if ( gesture == PIWI ):															# Pan Clockwise
					
					if (duty[curPi] <= 70): 
						
						duty[curPi] += 10
						
						ssh_string = "ssh " + curPi_name + " 'python /home/pi/scripts/moveMotor.py  " + str(duty[curPi]) + " 0 0' &"
						os.system(ssh_string)

					elif ( ( duty[curPi] > 70 ) and ( duty[curPi] < 80 ) ):
						duty[curPi] = 80
						ssh_string = "ssh " + curPi_name + " 'python /home/pi/scripts/moveMotor.py  " + str(duty[curPi]) + " 0 0' &"
						os.system(ssh_string)
						print("Motor is at limit.")
					
					t_endWaiting = time.time() + 1
					t_30_SLEEP = time.time() + 30
					
					
				elif ( gesture == PIWO ):														# Pan Counter Clockwise
					
					if ( duty[curPi] >= 30 ):
					
						duty[curPi] -= 10
						ssh_string = "ssh " + curPi_name + " 'python /home/pi/scripts/moveMotor.py  " + str(duty[curPi]) + " 1 0' &"
						os.system(ssh_string)
					
					elif ( ( duty[curPi] < 30 ) and ( duty[curPi] > 20 ) ):
						
						duty[curPi] = 20
						ssh_string = "ssh " + curPi_name + " 'python /home/pi/scripts/moveMotor.py  " + str(duty[curPi]) + " 1 0' &"
						os.system(ssh_string)
						print("Motor is at limit.")
						
					else:
						print("Motor is out of range. Cannot rotate CCW")
				
					t_endWaiting = time.time() + 1
					t_30_SLEEP = time.time() + 30
				
				elif ( gesture == FIST ):
					
					mode = WAITING
					print("SWITCHING MODE: WAITING\t\t\t\tConfidence Level: ", confidence)
					t_endWaiting = time.time() + 1
		
	return mode
	

def getConfidence(realTimeData, calData):
	
	matchCounter = 0
	
	'''
		calibrated: 823
		actual: 	832
		result: 	10 + 2 + 3 = 15
		
		calibrated: 781
		actual:		832
		result:		7
		
		calibrated: 231
		actual:		832
		result:		1 + 6 + = 7
	'''
	
	if (realTimeData[0] == calData[0]):
		matchCounter += 10
	if (realTimeData[0] == calData[1]):
		matchCounter += 7
	if (realTimeData[0] == calData[2]):
		matchCounter += 3
	
	if (realTimeData[1] == calData[0]):
		matchCounter += 4
	if (realTimeData[1] == calData[1]):
		matchCounter += 6
	if (realTimeData[1] == calData[2]):
		matchCounter += 2
		
	if (realTimeData[2] == calData[0]):
		matchCounter += 2
	if (realTimeData[2] == calData[1]):
		matchCounter += 3
	if (realTimeData[2] == calData[2]):
		matchCounter += 4
		
	return matchCounter

	
'''
	If the gesture is the same as the last one, increment the counter. If the gesture is different from the last gesture, 
	update the variable, lastGesture, and reset the counter. This allows us to wait for n counts of the same gesture before 
	considering a gesture valid. 
'''
def confirmGesture(gesture):
	global CONFIRM_COUNTER
	
	if ( confirmGesture.lastGesture != gesture ):
		confirmGesture.flag = False
		
	if ( confirmGesture.counter < CONFIRM_COUNTER ):
		confirmGesture.counter += 1
		confirmGesture.flag = False
	else:
		confirmGesture.lastGesture = gesture
		confirmGesture.counter = 0
		confirmGesture.flag = True
				
			
	return confirmGesture.flag

confirmGesture.flag = False																# static variable initialization for the above function
confirmGesture.counter = 0
confirmGesture.lastGesture = REST

if __name__ == '__main__':

	m = MyoRaw(sys.argv[1] if len(sys.argv) >= 2 else None)								# this has to come first, and proc_emg() second (see below)
	
	def proc_emg(emg, moving, times = []):												# data is sent in packets of two samples at a time. I *think* we only save half of these
		global calMode; global emg_data
		global fistCalData;	global piwiCalData;	global piwoCalData;
		
		emg = list(emg)																	# convert tuple to list
		emg_data = emg
		
		if ( mode == GETTINGCALDATA ):													# write calibration data to a global array
			
			if (calMode == FIST):
				fistCalData.append(emg_data)
			if (calMode == PIWI):
				piwiCalData.append(emg_data)
			if (calMode == PIWO):
				piwoCalData.append(emg_data)
	
	''' 
		INITIALIZATION	
		this code is only executed once
	'''
	m.add_emg_handler(proc_emg)
	m.connect()
	global GETTINGCALDATA; global CALIBRATING; global SLEEP; global WAITING; global DISPLAYCONTROL; global MOTORCONTROL; 
	global REST; global FIST; global PIWI; global PIWO; global calMode; global curPi; global CONFIRM_COUNTER;

	os.system("python displayControl.py &")												# initializes the display on every run
	
	confidenceArray = []
	
	curPi = 0
	gesture = REST														
	isResting = 0

	BUFFER_SIZE = 100																	# size of circular buffer 
	emg_buffer = ringBuffer(BUFFER_SIZE)
	counter = 0																			# counter 
	
	CONFIDENCE_LEVEL = 10																# allows for tuning. Max = 20. Min = 0. See getConfidence() 
	CONFIRM_COUNTER = 150																# number of samples of same gesture required to confirm a gesture
	SENSITIVITY = 75																	# upper and lower threshold = minValueFromCal +/- SENSITIVITY
	
	NUM_CALS = 4																		# this is always 1 greater than the number of calibrations
	
	CALIBRATION_SIZE = 500
	n = CALIBRATION_SIZE
	CSVFILE = "./adityaCal.csv"															# file to write/read calibration data from	
	minValueFromCal = 9999																# initially an arbitrarily large value
	iWantToCal = 0																		# set to '1' when switching users or when recalibration is needed
	calibrateFlag = 1
	
	if ( iWantToCal == 1 ):
		mode = GETTINGCALDATA
	else:
		mode = SLEEP																	# skip GETTINGCALDATA and CALIBRATING states
		
	os.system("ssh emgPi_1 'python /home/pi/scripts/initMotor.py 50' &")				# The ampersand is essential here. If this does not run in the background ... 
	os.system("ssh emgPi_2 'python /home/pi/scripts/initMotor.py 50' &")				# the bluetooth protocol fails and the system is frozen. 
	
	print("MOTORS INITIALIZED")
	os.system("clear")
	
	while True:																			# run the program indefinitely, or until user interruption
		m.run()
	
		emg_buffer.append(emg_data)
			
		if (counter >= BUFFER_SIZE * 2):												# there was an undiagnosed issue with 7 null data points causing havoc. 
																						# this ensures that those are gone before proceeding
			
			average = emg_buffer.getAvg()												# average value of each sensor in the buffer. [ 1 x 8 ]
			
			bufferAvg = np.mean(np.array(average))										# average value of the whole buffer. type: float, [1 x 1]
			
			maxGrouping = emg_buffer.getMaxGrouping()
			
			if ( mode >= SLEEP ):														# where the main gesture detection and control happens
			
				if ( calibrateFlag == 1 ):												# load saved cal data 
					with open(CSVFILE, 'rb') as csvfile:								# Example: [ 7, 6, 1]; [ 4, 2, 5]; [ 0, 2, 7]; [ 157.6, 157.6, 157.6]
						CalReader = csv.reader(csvfile, delimiter=',')
						i = 0
						for row in CalReader:
							savedCalData = np.genfromtxt(CSVFILE, delimiter=',')
					
					print("Calibration Data: \n", savedCalData)		
					print("MODE = SLEEP")
					calibrateFlag = 0
					fistGrouping = savedCalData[0]
					piwiGrouping = savedCalData[1]
					piwoGrouping = savedCalData[2]
					minValueFromCal = savedCalData[3,1]
			
				fistConfidence = getConfidence(maxGrouping, fistGrouping)
				piwiConfidence = getConfidence(maxGrouping, piwiGrouping)
				piwoConfidence = getConfidence(maxGrouping, piwoGrouping)
				
				confidenceArray = [fistConfidence, piwiConfidence, piwoConfidence]
				
				maxMatch = np.argmax(confidenceArray)									# index of the gesture that returned the most confidence
				maxConfidence = confidenceArray[maxMatch]								# confidence level of the most confident gesture
			
				if ( ( bufferAvg >= ( minValueFromCal + SENSITIVITY ) ) ):  
					if ( maxMatch == 0 ) and ( fistConfidence >= CONFIDENCE_LEVEL) :
						
						if ( confirmGesture(FIST) ):									# if we saw FIST for n times
							gesture = FIST
							print("\tFIST CONFIRMED\t\t\t\tConfidence Level: ", fistConfidence)
							
							isResting = 0
						
					elif ( maxMatch == 1 ) and ( piwiConfidence >= CONFIDENCE_LEVEL ):
											
						if ( confirmGesture(PIWI) ):									# if we saw PIWI for n times
							gesture = PIWI
							print("\tPIWI CONFIRMED\t\t\t\tConfidence Level: ", piwiConfidence)
							
							isResting = 0
						
					elif ( maxMatch == 2 ) and ( piwoConfidence >= CONFIDENCE_LEVEL ):
						
						if ( confirmGesture(PIWO) ):									# if we saw PIWO for n times
							gesture = PIWO
							print("\tPIWO CONFIRMED\t\t\t\tConfidence Level: ", piwoConfidence)

							isResting = 0
					else:
						if ( confirmGesture(REST) ):									# if we saw REST for n times
							gesture = REST
							print("\n\n\tMOTION DETECTED BUT NO GESTURE MATCH: REST ASSUMED")
							print("\n\tMinimum Accepted Confidence: ", CONFIDENCE_LEVEL)
							print("\tFIST Confidence: ",fistConfidence, "\tPIWI Confidence: ",piwiConfidence, "\tPIWO Confidence: ",piwoConfidence)
							print("\tStill in mode: ", modeString[mode])
							print("\n\n")						
						
				elif ( (bufferAvg < (minValueFromCal - SENSITIVITY)) ): #isResting or 
				
					#print("REST CONFIRMED")
					gesture = REST
					isResting = 1
					
				#else:
					
					# print("UNKNOWN")	
					# print("Sensitivity: ", SENSITIVITY)
					# print("minValueFromCal: ", minValueFromCal)
					# print("Buffer average: ", bufferAvg)
				
				mode = controlLogic(mode, gesture, maxConfidence)						# get new mode

			''' 
				CALIBRATION
				note: this can probably be put into a function later. Maybe not all of it, but enough that it becomes a little easier to follow
			'''               
			if ( ( mode == GETTINGCALDATA ) and ( calMode < NUM_CALS ) ):
				
				if (n >= CALIBRATION_SIZE):												
					
					n = 0																# reset calibration timer
					print("Cal Mode = " + gestureString[calMode])
					print("Hold a " + gestureString[calMode] + " until told otherwise")
					calMode += 1
					# time.sleep(2)          											# WARNING: THIS BREAKS THE CODE! # sleep to give user time to switch to next gesture
				
				n += 1				
				if (bufferAvg < minValueFromCal):										# this gets the minimum 8-sensor average from the time that calibration was run
					minValueFromCal = bufferAvg											# it sets the threshold that separates gestures from resting. 
            
			else:
				
				if ( calibrateFlag == 1 ):
					mode = CALIBRATING

				gesture = REST
				mode = controlLogic(mode, gesture, 0)
           
				
			if ( mode == CALIBRATING ) : 
				
				print("mode = CALIBRATING")
				fistCal = Calibrate()
				fistGrouping = fistCal.getMaxGrouping(fistCalData)					
					
				piwiCal = Calibrate()
				piwiGrouping = piwiCal.getMaxGrouping(piwiCalData)
				
				piwoCal = Calibrate()
				piwoGrouping = piwoCal.getMaxGrouping(piwoCalData)
				
				minValueFromCalArray = [minValueFromCal,minValueFromCal,minValueFromCal]
				
				with open(CSVFILE, 'w') as csvfile:
					writer = csv.writer(csvfile)
					writer.writerow(fistGrouping)
					writer.writerow(piwiGrouping)
					writer.writerow(piwoGrouping)
					writer.writerow(minValueFromCalArray)
					
				calibrateFlag = 0
				mode = SLEEP

				print("Fist Group: ", fistGrouping)
				# print(fistCalData)
				print("Piwi Group: ", piwiGrouping)
				# print(piwiCalData)
				print("Piwo Group: ", piwoGrouping)
				
			
		else:																			# Runs until data is guaranteed to be good
		
			counter += 1
			# print(counter, "Data contains null values\n")
			
