'''
AUTHOR: Aditya Patel
DATE CREATED: 2018-04-08
LAST MODIFIED: 2018-04-09
PLATFORM: Raspberry Pi 3B, Raspbian Stretch Released 2017-11-29
PROJECT: EMG Human Machine Interface
ORGANIZATION: Bradley University, School of Electrical and Computer Engineering
FILENAME: calibrate.py
DESCRIPTION:
	Calibration class. Create a unique Calibrate() object in the parent function for each gesture. 
	Primary use of this class is to get the top three sensor groupings in the calibration data, 
	i.e., the three trios of consecutive sensors with the highest average EMG value. 
	
	Ex:
		If the calibration data is: 
			[ 99    100    32    03    14    16    42    95 ]
			   
		index: 0      1     2     3     4     5     6     7
		
		The top three sensor groups would be, in order, 
			[ 701, 670, 012 ]
			
		Giving the return:
			[ 7, 6, 0]
'''

import numpy as np

class Calibrate():

	def __init___(self):
		
		self.size()
		self.data = []
		self.sums = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.avg = []
		# self.avg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])					# this has to be a numpy array to use the 'divide' function below
		self.groupingAvg = []
		
	''' 
		Sets the class variable, data, equal to the calibration data. 
	'''
	def setData(self, calData):
	
		self.data = calData
		
	'''
		Computes an average down each column of data. 
		writes to self.avg, 1 x 8 array containing average value of each sensor
	'''
	def getAvg(self):
	
		self.sums = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]							# reset sums to prevent it from accumulating forever. This is NOT elegant or efficient
		self.avg =  np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		for r in range(0,len(self.data)):				
			for c in range(0,8):				
				self.sums[c] = self.sums[c] + self.data[r][c]

		np.divide(self.sums, float(len(self.data)), out = self.avg)						# compute the average by dividing the sums by the size of the data array
		#return self.avg
	
	'''
		Function to compute the average value of each of the eight groupings of three sensors. 
		@return none, only writes to class variable, groupingAvg 
				[ 123    234    345    456    567    678    781    812]		
				where each of these is the average of the sensors
	''' 
	def getGroupingAvg(self):
		
		self.groupingAvg = [0, 0, 0, 0, 0, 0, 0, 0]
		
		for startIndex in range(0, 8):
			sum = 0

			for i in range(startIndex, startIndex + 3):
				
				if ( i > 7 ):
					i %= 8																# if i exceeds the range of the data, do the modulus operator. This allows for groupings 781 and 812 to work. 
				sum += self.avg[i]
				i += 1
			
			self.groupingAvg[startIndex] = sum / 3.0
			
	
	'''
		Function to compute the three highest sensor groups. 
		@return maxGrouping, [1 x 3] integer list of the index of the top three sensor groups in the calibration data
		@example maxGrouping = myCalibrationObject.getMaxGrouping(gestureCalibrationData) --> maxGrouping: [ 6, 5, 0]
	''' 
	def getMaxGrouping(self,calData):
	
		maxGrouping = [0, 0, 0]
		self.setData(calData)
		self.getAvg()
		self.getGroupingAvg()
		
		array = np.array(self.groupingAvg)												# REFERENCE: https://stackoverflow.com/questions/5284646/rank-items-in-an-array-using-python-numpy
		temp = array.argsort()
		ranks = np.empty_like(temp)
		ranks[temp] = np.arange(len(array))
		
		maxGrouping[0] = int( np.where(ranks == 7)[0] )
		maxGrouping[1] = int( np.where(ranks == 6)[0] )
		maxGrouping[2] = int( np.where(ranks == 5)[0] )
		
		return maxGrouping
		
'''
	Used for Testing/Debugging Purposes 
'''
if __name__ == '__main__':
	
	t1 = []
	#t2 = [0, 1, 2, 3, 4, 5, 6, 7]  
	t2 = [100.0, 100.0, 7.0, 7.0, 0.0, 0.0, 0.0, 100.0]
	
	t1.append(t2)
	t1.append(t2)
	t1.append(t2)
	cal = Calibrate()
	maxGrouping = cal.getMaxGrouping(t1)
	
	
	print(maxGrouping)