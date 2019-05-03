from qlbot import *
from qlbotconf import *
from os import getcwd
import time
import csv
import random as rand
import io

cwd = "/home/student/catkin_ws/src/qlearning_project/src/qlearning_project"

################################### Database operation ###################################

def sort_keys(Q):
	return sorted([ (val[0][0], val[0][1], val[1]) for val in list(Q.keys()) ])

def WriteData(self_, filename, OLD_DATA):

	if DISABLE_Qvl_SAVING : return -1

	with open(cwd + FolderName + filename + 'E%d' % self_.Episode + '.csv' , 'wb') as f:

		### Calculate explore rate
		state_step_explored = sum( [ 1 if i else 0 for i in self_.Q_stepTaken.values() ] ) / (( (self_.StateSlice+1) ** self_.LaunchNums ) * self_.ActionNums * OMEGAAUGNUM)

		### Print Params to the First 4 Rows
		writer=csv.writer(f)
		write_li = ['','angularSpeed', angularSpeed, 'linearSpeed', linearSpeed]
		try: 
			if RECORD_PREVIOUS_PARAM : write_li = write_li + ['','','','', 'OLD_DATA'] + OLD_DATA
		except TypeError : write_li = write_li + ['', 'OLD_DATA', 'Err']
		writer.writerow([filename, 'episodes', self_.Episode, 'avg_step', "%.2f" % self_.avg_step, 'avg_reward', "%.2f" % self_.avg_reward, 'Epsilon', self_.EPSILON, 'Explored %', "%.3f%%" % float(100 * state_step_explored)])
		writer.writerow(['','ScanSlice' , self_.ScanSlice, 'StateSlice', self_.StateSlice, 'LaunchNums', self_.LaunchNums, 'ALPHA', self_.ALPHA, 'GAMMA', self_.GAMMA, 'SLEEP_TIME', self_.SLEEP_TIME, 'ActionNums', self_.ActionNums])
		writer.writerow(write_li)
		write_li=['','Crush thre',SensingThreshold__[0], 'Sensing thre']
		for i in range(1, StateSlice): write_li.append(SensingThreshold__[i])
		writer.writerow(write_li)

		### Print Empty Row and Label Row to the Next 2 Rows
		write_li = ['','','State_om','State_dist']
		for i in range(0,ActionNums): write_li.append('Action '+ str(i))
		write_li.append('Action Taken 0')
		for i in range(1,ActionNums): write_li.append('A T '+ str(i))
		writer.writerow("")
		writer.writerow(write_li)

		### Q-table and Q_stepTaken-table start printing from here
		outputData = []
		doneList = []		
		for index in sort_keys(self_.Q):
			om, di, j = index
			i = om, di
			if i not in doneList:
				doneList.append(i)
				subOutput = ['','state', float(i[0]), float(i[1])]
				QST = [int(float(self_.Q_stepTaken.setdefault((i, k), 0))) for k in range(0, ActionNums)]
				if all( i == 0 for i in QST ) : continue
				for k in range(0, ActionNums):
					subOutput.append( float(format( float(self_.Q.setdefault((i, k), 0)) , '0.5f')) )
				for k in range(0, ActionNums):
					subOutput.append(float(self_.Q_stepTaken.setdefault((i, k), 0)))
				outputData.append(subOutput)
		# print(outputData)
		writer.writerows(sorted(outputData))
	print( color_Y + "Write Data Done!" + color_W )

	f.close()
	

def WriteLog(self_, filename, episode, step, reward):
	if DISABLE_LOG_SAVING : return -1	
	with open( cwd + FolderName + 'log/' + 'log_' + filename + '_E' + str(episode) +'.csv', 'wb') as f:
		writer = csv.writer(f)
		if step == MAX_STEP : writer.writerow([ ' ', ' ', ' ', 'log' + filename, 'ep ', episode, 'step ',  step, 'r ', "%.3f" % reward, 'ALPHA', "%.4f" % self_.ALPHA])
		else : writer.writerow([ ' ', ' ', ' ', 'log' + filename, 'ep ', episode, 'step ',  step, 'r ', "%.3f" % reward])
		write_li = [ ' ', ' ', ' ','','', 's', 'a', 's_', '--' ]
		for i in range(0, ActionNums):
			write_li.append('Q[s, a%d]' % i)
		write_li.append('--')
		for i in range(0, ActionNums):
			write_li.append('ST[s, a%d]' % i)
		write_li.append('--')
		write_li.append('policy')
		write_li.append('reward')
		writer.writerow(write_li)
		for i in self_.Q_traj : writer.writerow(i)
		writer.writerow(['end'])
	print( color_Y + "Write Log Done!" + color_W )
	f.close()


def ReadData(self_, filename):
	if filename == None :  ### If Not Using Old Data
		p_INFO = None
		print("No Data Input")
		self_.Q                  = {}
		self_.Q_stepTaken        = {}
		self_.Episode            = 0
		self_.avg_step           = 0
		self_.avg_reward         = 0
		self_.EPSILON            = EPSILON 		
		self_.ScanSlice          = ScanSlice
		self_.StateSlice         = StateSlice
		self_.LaunchNums         = LaunchNums
		self_.ALPHA              = ALPHA
		self_.GAMMA              = GAMMA
		self_.SLEEP_TIME         = SLEEP_TIME
		self_.angularSpeed       = angularSpeed
		self_.linearSpeed        = linearSpeed
		self_.ActionNums         = ActionNums
		self_.SensingThreshold__ = SensingThreshold__	
	else:
		p_INFO=[]
		print('Read File')
		try:
			with open( cwd + FolderName + 'load/' + filename ) as csvfile:

				ReadinQ = {}
				ReadinQST = {}
				readCSV = csv.reader( csvfile, delimiter = ',' )

				""" 
				###Data Format
				2017-08-17_10-55-17.csv,episodes,1150,avg_step,611.16,avg_reward,-405.65,Epsilon,0.0001,Explored State,59
				,ScanSlice,12,StateSlice,3,LaunchNums,3,ALPHA,0.3,GAMMA,0.5,SLEEP_TIME,0.25,ActionNums,3
				,angularSpeed,0.45,linearSpeed,0.12
				,Crush thre,0.39,Sensing thre,0.8,1.4
				"""
					
				row = [ item for item in iter(readCSV) ]

				### Read Parameters of The Input File
				p_Episode     =  int(row[0][2])
				p_avg_step    =  float(row[0][4])
				p_avg_reward  =  float(row[0][6])
				p_Epsilon     =  float(row[0][8])

				p_ScanSlice   =  int(row[1][2])
				p_StateSlice  =  int(row[1][4])
				p_LaunchNums  =  int(row[1][6])
				p_ALPHA       =  float(row[1][8])
				p_GAMMA       =  float(row[1][10])
				p_SLEEP_TIME  =  float(row[1][12])
				p_ActionNums  =  int(row[1][14])

				p_AS          =  float(row[2][2])
				p_LS          =  float(row[2][4])

				p_CT          =  float(row[3][2])
				p_ST          = [float(row[3][i+4]) for i in range(0, p_StateSlice-1)]
				p_ST          = [p_CT] + p_ST

				p_INFO = [[p_Episode, p_avg_step, p_avg_reward, p_Epsilon]]
				p_INFO.append([p_ScanSlice, p_StateSlice, p_LaunchNums, p_ALPHA, p_GAMMA, p_SLEEP_TIME, p_ActionNums])
				p_INFO.append([p_AS, p_LS])
				p_INFO.append(p_ST)


				### Read Q and Q_stepTaken
				if OMEGAAUG is True:
					for data in range( 6, len(row) ) :
						for i in range( 4, 4 + p_ActionNums ): 
							ReadinQ[((int(float(row[data][2])),int(float(row[data][3]))), i-4)] = float(row[data][i])
						for i in range( 4 + p_ActionNums, 4 + 2 * p_ActionNums ): 
							ReadinQST[((float(row[data][2]),int(float(row[data][3]))), i-4-p_ActionNums)] = int(float(row[data][i]))
				else : 
					input('Not Done Yet. ReadData()@record_data.py')
				
				### Initialize Params and Values in class QLbot

				self_.Q                  = ReadinQ      if (USE_DATA['Q']                  and UPP) else {}
				self_.Q_stepTaken        = ReadinQST    if (USE_DATA['Q_stepTaken']        and UPP) else {}
				self_.Episode            = p_Episode    if (USE_DATA['Episode']            and UPP) else 0
				self_.avg_step           = p_avg_step   if (USE_DATA['avg_step']           and UPP) else 0
				self_.avg_reward         = p_avg_reward if (USE_DATA['avg_reward']         and UPP) else 0
				self_.EPSILON            = p_Epsilon    if (USE_DATA['EPSILON']            and UPP) else EPSILON 		
				self_.ScanSlice          = p_ScanSlice  if (USE_DATA['ScanSlice']          and UPP) else ScanSlice
				self_.StateSlice         = p_StateSlice if (USE_DATA['StateSlice']         and UPP) else StateSlice
				self_.LaunchNums         = p_LaunchNums if (USE_DATA['LaunchNums']         and UPP) else LaunchNums
				self_.ALPHA              = p_ALPHA      if (USE_DATA['ALPHA']              and UPP) else ALPHA
				self_.GAMMA              = p_GAMMA      if (USE_DATA['GAMMA']              and UPP) else GAMMA
				self_.SLEEP_TIME         = p_SLEEP_TIME if (USE_DATA['SLEEP_TIME']         and UPP) else SLEEP_TIME
				self_.angularSpeed       = p_AS         if (USE_DATA['angularSpeed']       and UPP) else angularSpeed
				self_.linearSpeed        = p_LS         if (USE_DATA['linearSpeed']        and UPP) else linearSpeed
				self_.ActionNums         = p_ActionNums if (USE_DATA['ActionNums']         and UPP) else ActionNums
				self_.SensingThreshold__ = p_ST         if (USE_DATA['SensingThreshold__'] and UPP) else SensingThreshold__	
					



				print('Params and Values Initialized')

			csvfile.close()

		except IOError:
			
			self_.start = False
			return None

	self_.start = True
	return p_INFO
