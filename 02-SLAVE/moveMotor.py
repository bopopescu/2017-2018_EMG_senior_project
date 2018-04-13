'''
AUTHOR: Aditya Patel and Jim Ramsay
DATE CREATED: 2018-04-05
LAST MODIFIED: 2018-04-07
PLATFORM: Raspberry Pi 3B, Raspbian Stretch Released 2017-11-29
PROJECT: EMG Human Machine Interface
ORGANIZATION: Bradley University, School of Electrical and Computer Engineering
FILENAME: moveMotor.py
DESCRIPTION: 
	Script to move the connected servo motor in a specified direction by 0.01% duty cycle. 

NOTE:

	CW  --> INCREASE PWM
	CCW --> DECREASE PWM
	
	20% --> +90 deg
	80% --> -90 deg
	
USAGE:		

	ssh emgPi_1 'python /home/pi/scripts/moveMotor.py PWMdir isLastTime'
	
	Command Line Arguments:
		
		PWMdir		integer	 0 --> CW		1 --> CCW	others --> not recognized, do nothing
		isLastTime	boolean	True --> execute IO cleanup		False --> do not cleanup, run normally
	
'''

import RPi.GPIO as IO
import sys
import time

global initialDuty
initialDuty = 50

class Motor:
	
	global initialDuty
	def __init__(self, PWMdirection, startPWM):

		IO.setwarnings(False)
		IO.setmode(IO.BOARD)

		IO.setup(12, IO.OUT)								# set GPIO pins to Output mode
		self.p = IO.PWM(12,350)								# set pin 12 to 350Hz pwm output
		self.PWMdir = PWMdirection
		self.duty = startPWM
		self.p.start(startPWM)								

	def ccw(self):
		if (self.duty >= 20) and (self.duty <= 70) :
			self.duty += 10
			# print("ccw")
			# print(self.duty)
			self.p.ChangeDutyCycle(self.duty)
			time.sleep(0.01)

	def cw(self):
		if ( self.duty >= 30 ) and ( self.duty >= 80 ) :
			self.duty -= 10.0
			# print("cw")
			# print(self.duty)
			self.p.ChangeDutyCycle(self.duty)
			time.sleep(0.01)
			
	def cleanup(self):
		self.p.stop()
		IO.cleanup()
	
if __name__ == '__main__':
	
	# moduleName = sys.argv[0]
	startPWM = float(sys.argv[1])
	PWMdir = int(sys.argv[2])									
	isLastTime = int(sys.argv[3])
	
	# print("PWM Direction: ", type(PWMdir))

	mtr = Motor(PWMdir, startPWM)
	time.sleep(1)
	# if (PWMdir == 0):
		# mtr.cw()
		# time.sleep(1)
	# elif (PWMdir == 1):
		# mtr.ccw()
		# time.sleep(1)
	# if (isLastTime):
		# mtr.cleanup()
	
	
	# t_end = time.time() + 10
	
	# if (isLastTime == 1):
			# mtr.cleanup()
			
	# while (time.time() < t_end):
		# if (PWMdir == 0):
			# mtr.cw()
		# elif (PWMdir == 1):
			# mtr.ccw()
	
	
