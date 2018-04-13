'''
AUTHOR: Aditya Patel and Jim Ramsay
DATE CREATED: 04/01/2018
LAST MODIFIED:  2018-04-09
PLATFORM: Raspberry Pi 3B, Raspbian Stretch Released 2017-11-29
PROJECT: EMG Human Machine Interface
ORGANIZATION: Bradley University, School of Electrical and Computer Engineering
FILENAME: ringBuffer.py
DESCRIPTION:
	Class that implements a ring/circular buffer to hold the emg data. It stores data until full, then
	overwrites the oldest element every time. It also has a method to take an average of the last n 
	data points. 

KNOWN FLAW:
	Instead of only ignoring the oldest element when computing the average, I flush the entire buffer. 
	Then the full n-length sum is taken. This is grossly inefficient, but was not found to be a bottleneck
	in implementation. Thus, it was ignored. 
	
EDIT HISTORY:
	20180409 -- Added functions to compute groupingAvg and maxGrouping. The groupings are as follows: 
					
					[ 123    234    345    456    567    678    781    812 ]
					
				In main, this allowed us to calculate the three groupings with the highest average sensor value. 
'''
import numpy as np

class ringBuffer:
	def __init__(self,size_max):														# Constructor
		self.max = size_max
		self.data = []
		self.sums = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]							
		self.avg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])					# this has to be a numpy array to use the 'divide' function below
		self.full = False
		self.groupingAvg = []
	class __Full:																		# sub-class that implements a full buffer

		def append(self, x):
			self.data[self.cur] = x														# append an element, overwriting the oldest one
			self.cur = (self.cur + 1) % self.max										# cycle 'cur' from 0 to self.max

		def getAvg(self):
			self.sums = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]						# reset sums to prevent it from accumulating forever. This is NOT elegant or efficient
			for r in range(0,self.max):				
				for c in range(0,8):				
					self.sums[c] = self.sums[c] + self.data[r][c]
	
			np.divide(self.sums, float(self.max), out = self.avg)						# compute the average by dividing the sums by the size of the data array
			return self.avg

		def get(self):																	# return list of elements from oldest to newest
			return self.data[self.cur:] + self.data[:self.cur]
				
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
						i %= 8															# if i exceeds the range of the data, do the modulus operator. This allows for groupings 781 and 812 to work. 
					sum += self.avg[i]
					i += 1
				
				self.groupingAvg[startIndex] = sum / 3.0
				
		'''
			Function to compute the three highest sensor groups. 
			@return maxGrouping, [1 x 3] integer list of the index of the top three sensor groups in the ring buffer
			@example maxGrouping = myRingBuffer.getMaxGrouping() --> maxGrouping: [ 6, 5, 0]
		''' 
		def getMaxGrouping(self):
	
			maxGrouping = [0, 0, 0]
			
			self.getAvg()
			self.getGroupingAvg()
			
			array = np.array(self.groupingAvg)											# REFERENCE: https://stackoverflow.com/questions/5284646/rank-items-in-an-array-using-python-numpy
			temp = array.argsort()
			ranks = np.empty_like(temp)
			ranks[temp] = np.arange(len(array))
			
			maxGrouping[0] = int( np.where(ranks == 7)[0] )
			maxGrouping[1] = int( np.where(ranks == 6)[0] )
			maxGrouping[2] = int( np.where(ranks == 5)[0] )
			
			return maxGrouping
			
			
	def append(self,x):																	# append an element to the end of the buffer until it is full
		self.data.append(x)

		if len(self.data) == self.max:
			self.cur = 0
			self.full = True
			self.__class__ = self.__Full												# Permanently change class from not full to full

	def get(self):																		# return list of elements from oldest to newest
		return self.data


'''
	Used for Testing/Debugging Purposes 
'''
if __name__=='__main__':
	x = ringBuffer(3)
	print "average: ", x.avg
	print "sums: ", x.sums
	emg1 = [1,1,1,1,1,1,1,1]
	emg2 = [2,2,2,2,2,2,2,2]
	emg3 = [3,3,3,3,3,3,3,3]
	x.append(emg1); x.append(emg2); x.append(emg3);
	x.append(emg3); x.append(emg3);x.append(emg3);
	average = x.getAvg()

	print "Average: ", average






