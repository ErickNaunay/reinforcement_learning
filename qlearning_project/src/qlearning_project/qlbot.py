#!/usr/bin/env python
import record_data as dt
from bumperaction import BumperAction
from qlbotconf import *
from ql_adj_func import *
import random as rand
import rospy
import message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import math
import os
import time
import csv
import copy
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class QLbot:

	"""
		@ BRIEF  : Object Constructor
		@ INPUT  : None
		@ RETURN : None
	"""
	def __init__(self, DataName, simulation):

		self.OLD_DATA = dt.ReadData(self, DataName)
		self.bumper_sensor = BumperAction()
		self.simulation = simulation

		if self.start: ### self.start = False if LoadName is Not None and Loaded Data Not Found

			if self.simulation:
				Gazebo_reset_srv()
			
		#	if not DISABLE_MODEL_RESET: 
		#		Reset_Model_srv()

		### For Decision Making
			self.state = 0  # QLbot.state is a tuple in the form of (omega_state, distance_state)
			self.step_saver = [] # Also Init in 'For Matplotlin' Section Since Plot Part Can Be Disabled
			self.dist = [0] * self.LaunchNums
			self.dist_cat = [0] * self.LaunchNums
			self.last_dist_cat = [0] * self.LaunchNums
			self.omega = 0
			self.previous_dist_cat_for_risk_eva = 0

		### For Log Saving
			self.single_episode_reward = self.total_reward = 0
			self.step = 0
			self.Q_traj = []
			self.Q_traj_n = 0

		### For ROS Operation
			self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 5)
			self.pub_rate = rospy.Rate(30)
			if self.simulation:
				self.sub_scan = message_filters.Subscriber('/scan',LaserScan)
			else:
				self.sub_scan = message_filters.Subscriber('scan',LaserScan)

			self.ts_scan = message_filters.ApproximateTimeSynchronizer([self.sub_scan],10, 0.1,allow_headerless = True)
			self.ts_scan.registerCallback(self.cb_scan)
			self.sub_odom = message_filters.Subscriber('/odom',Odometry)
			self.ts_odom = message_filters.ApproximateTimeSynchronizer([self.sub_odom],10, 0.1,allow_headerless = True)
			self.ts_odom.registerCallback(self.cb_odom)
			self.twist = Twist()

		### For Matplotlib
			if not DISABLE_PLT_SHOWING or not DISABLE_PLT_SAVING:
				plt.ion()
				self.fig = plt.figure()
				self.step_saver = [0]
				self.avg_step_saver = [0]
				self.max_rate_saver = [0]
				self.max_step_times = 0
				self.max_step_recorder = [0]
				self.max_step_per20 = [0]
				self.first_max = -1
				self.wait_time_saver = [0.2]
				self.t0 = None
				self.abort_step_saver=[0]
				self.abort_this_ep = 0
				self.DrawPlt()

	"""
		@ BRIEF  : First wait until TIME_GAP reached, then get sensors' values, 
		@ BRIEF  : Calculates those data into states to return.
		@ INPUT  : None
		@ RETURN : CurrentState (om_state, dist_state)
		@ ANNOT  : This Function Won't Modify self.state Directly
	"""
	def GetCurrentState(self): 

		### Save Prvious State For Risk Evaluator
		self.previous_dist_cat_for_risk_eva = copy.deepcopy(self.dist_cat)


		### Wait For Time Gap Reached
		### If Time Taken Between 2 GetCurrentState Too short ( less than TIME_GAP ) , Go Into The Loop ( Wait for TIME_GAP_WAIT )
		### If Time Taken is Too Long, over TIME_GAP_MAX, Q Table Will Not Be Modified After Current Step ( Wrote in self.UpdateQ() )
		### For Better Displaying, Those Episode Took Over TIME_GAP MAX Will Be Plot As TIME_GAP_ABORT
		if self.t0 == None: self.t0 = rospy.get_time()
		else:
			wait_nums = 0
			while rospy.get_time()-self.t0 < TIME_GAP :
				if wait_nums is 0 : print "Waiting for time gap reached...",
				wait_nums += 1
				rospy.sleep(TIME_GAP_WAIT)
			if wait_nums is not 0: print("%d Times. Continue to get state." % wait_nums)
			gap = rospy.get_time()-self.t0 if rospy.get_time()-self.t0 <= TIME_GAP_ABORT else TIME_GAP_ABORT
			self.wait_time_saver.append(gap)
			self.t0 = rospy.get_time()


		### (StateSlice+1)-to-decimal conversion
		itr = 1 ; current_dist_state = 0 ; self.dist_cat = [0] * self.LaunchNums
		if len(self.dist) is self.LaunchNums:
			for i in range(0, self.LaunchNums):
				if self.dist[i] < self.SensingThreshold__[0]:
					self.dist_cat[i] = -1
					current_dist_state += itr * self.StateSlice
				elif self.dist[i] > self.SensingThreshold__[self.StateSlice-1]: self.dist_cat[i] = 0
				else:
					for j in range(1, self.StateSlice):
						if self.dist[i] <= self.SensingThreshold__[j]:
							self.dist_cat[i] = self.StateSlice - j
							current_dist_state += itr * self.dist_cat[i]
							break
				itr *= (self.StateSlice+1)
		else : rospy.sleep(0.18)

		### Calculate Omega State, Function Defined in ql_adj_func.py
		current_om_state = Cal_OM_State(self.omega, self.angularSpeed)


		### Print To Screen
		print( "\033[35mGCS: self.dist = ", [ float("%.2f" % i) for i in self.dist ] , ', self.omega = ', self.omega , '\033[0m' )
		print( "\033[32mGCS: self.dist_cat = " , self.dist_cat , ', om_state = ', current_om_state, '\033[0m' )

		return (current_om_state, current_dist_state)



	"""
		@ BRIEF  : Choose the next action based on Q table and epsilon-greedy policy,
		@ INPUT  : Current state
		@ RETURN : action, action_policy_taken
		@ ANNOT  : ActionNums and EPSILON defined in ./QLbotConfig.py
		@ ANNOT  : Note that the argument "state" is a class member, but still needs to input from wherever it's called.
		         It was meant to maintain the consistency of the Q-updating formula and the code
	"""
	def ChooseAction(self, current_state):

		### Update EPSILON IF NEEDED, Function Defined in qlbotconf.py
		self.EPSILON = reduce_epsilon(self.Episode, self.EPSILON) # Defined in qlbotconf.py

		### Get Data In Q Table & stepTaken Table Of The Input State
		curr_om_state, curr_dist_state = current_state

		### Decisions Are Made Here
		### Randomly Give Unexplored States A Q-value, RANDOM_Q Defined In qlbotconf.py
		allState = [float(self.Q.setdefault((current_state, i), RANDOM_Q * float(rand.random()))) for i in range(0,self.ActionNums)]
		stepTaken = [int(self.Q_stepTaken.setdefault((current_state, i), 0)) for i in range(0,self.ActionNums)]
		if allState[1:] == allState[:-1]:                     ### if all Q-value are identical, choose the less taken one
			#                                                 ###### if all steps were taken equaly, choose by random 
			if stepTaken[1:] == stepTaken[:-1] or all( i >= 100 for i in stepTaken):	          
				action = rand.randrange(0, self.ActionNums)
				action_policy_taken = "All rand."
			else:                                             ###### choose one of the least taken steps by random
				min_ = [ i for i in range(0, self.ActionNums) if stepTaken[i] == min(stepTaken) ]
				action = min_[rand.randrange(0, len(min_))]
				action_policy_taken="Take less explored action."
		elif rand.random()<self.EPSILON:                      ### EPSILON part of the Epsilon-Greedy Algo.
			action = rand.randrange(0, self.ActionNums)
			action_policy_taken = "Epsilon = " + str(self.EPSILON)
		else:                                                 ### Greedy			
			max_ = [ i for i in range(0, self.ActionNums) if allState[i] == max(allState) ]
			if len(max_) == 1 :                               ###### Only one value that matches the max value
				action = max_[0]
				action_policy_taken = "Greedy"
			else:                                             ###### Several values match the max value
				min_ = [i for i in max_ if stepTaken[i] == min([stepTaken[j] for j in max_])]
				if all( i >= 100 for i in stepTaken):         ###### If all states are explored over 100 times than go absolutely random
					action = max_[rand.randrange(0, len(max_))]
					action_policy_taken = "Greedy+R"
				else:                                         ###### choose one of the least taken steps by random
					action = min_[rand.randrange(0, len(min_))]
					action_policy_taken = "Greedy+m"


		print "CA: State = ",
		print(self.state)

		print "CA: allState = ",
		print([ float(format(i, '0.3f')) for i in allState ])

		print "CA: stepTaken = ",
		print(stepTaken)
		
		print "CA: action, action_policy_taken  ",
		print(action,',', action_policy_taken)

		return action, action_policy_taken



	"""
		@ BRIEF  : Move to next state, get new state, s', write log data and return the reward.
		@ INPUT  : Action to execute, action policy ( for log recording )
		@ RETURN : state_prime( i.e. s'), reward    as a tuple
	"""
	def MoveToNextStep( self, action, action_policy ):
	##########################################################    Bad State Or Not   ##################################################
		### When Getting Into A Bad State, Spin And Search For Next State if DISABLE_SEARCHING_WITH_SPIN is False
		DeadEnd = False
		if not DISABLE_SEARCHING_WITH_SPIN:
			last_state = self.state
			turn_direction = 'CW' if self.Episode%2 == 0 else 'CCW'

			### Conditions to take spinning as an action
			### is_bad_state Defined in ql_adj_func.py
			if is_bad_state(self):
				originAction = action
				DeadEnd = True
				while self.state[1] == last_state[1] :     ### Stop spinning when dist_states are different
					action_policy = 'DeadEnd'
					print(color_BP + 'DeadEnd!!!! Searching for Next State. Direction: '+ turn_direction , color_W)
					### Spin in different direction in order to explore more states
					if turn_direction == 'CW' : self.Pub_Twist(0, -self.angularSpeed/2)
					else : self.Pub_Twist(0, self.angularSpeed/2)
					rospy.sleep(self.SLEEP_TIME)
					self.state = self.GetCurrentState()
					print('After Spin, new state =' , self.state)
					self.chooseAction(self.state)
				action = originAction
			else : DeadEnd = False
		else : DeadEnd = False
	##########################################################     Public Action     ##################################################
		if not MANUAL and not DeadEnd:
			self.Pub_Twist(self.linearSpeed, - self.angularSpeed + 2 * self.angularSpeed / (self.ActionNums-1) * action )
			rospy.sleep(self.SLEEP_TIME)
		
		state_prime = self.GetCurrentState()
		self.step += 1
	##########################################################   Reward Calculation  ##################################################
		reward = 0 ; isEnd = False
		obstacles = [self.dist_cat.count(i) for i in range(1,self.StateSlice)] ### The less the better  ### NOT VERY STRAIGHT FORWARD, MAY NEED TO MODIFY 
		bump = self.dist_cat.count(-1)
		
		print 'MTNS: obstacles = ',
		print obstacles, ', bump = ',
		print bump, ', step = ',

		if   self.step == 1: 
			print '\033[0m',
		elif self.step >= self.avg_step: 
			print '\033[1;33m',
		elif self.step <  self.avg_step: 
			print '\033[1;34m',

		print( str(self.step) + color_W)

		################################################ Came Out From A Bad State, Not End
		if DeadEnd : isEnd = False ; reward = -1  
		################################################ Reached MAX_STEP, End
		elif self.step >= MAX_STEP:
			print("MAX STEPPPPPPP") 
			isEnd = True
			reward = 0
			if ENABLE_ALPHA_DESCEND : self.ALPHA *= ALPHA_DESCEND
		################################################ Bump Into Something, End
		elif self.bumper_sensor.get_bumper_collision():
			print("BUMPEEEEER")
			isEnd = True
			reward = -1
		elif ep_terminate(bump, obstacles, self.dist_cat, self.LaunchNums, self.StateSlice): ### defined in qlbotconf.py
			print("STATEEEEE, ep_terminate")
			isEnd = True
			reward = -1
		################################################ Nothing Happen, Not End
		###### Evaluate Risk Of Current State, s, And Next State, s', 
		###### Then Decide Reward Accordingly
		### risk_evaluator and reward_giver defined in ql_adj_func.py
		else:  
			isEnd = False
			if self.step != 1:
				risk_now  = risk_evaluator(self, self.previous_dist_cat_for_risk_eva)
				risk_new  = risk_evaluator(self, self.dist_cat)
				reward = reward_giver(risk_now, risk_new, self.state, state_prime)
				print("risk: ", risk_now, risk_new , "reward =", reward)
			elif self.step == 1 : reward = 0

	########################################################## Handling Informations #############################################

		self.single_episode_reward += reward

		################################################ Log Info Handling
		self.Q_traj_n += 1
		LogQ = ["%.3f" % float(self.Q[(self.state, i)]) for i in range(0, self.ActionNums)]
		LogQST = [self.Q_stepTaken[(self.state, i)] for i in range(0, self.ActionNums)]
		new_traj = ['Episode', self.Episode+1, 'step', self.step , 'state', self.state, action, state_prime, '--'] + LogQ + ['--'] + LogQST + ['--', action_policy, reward]
		if self.Q_traj_n == 1  or  new_traj[5] != new_traj[7] or isEnd or reward == -1 : self.Q_traj.append( new_traj )

		if isEnd: 
			self.record_and_print_data_to_screen()

		return isEnd, state_prime, reward


	"""
		@ BRIEF  : Draw plot using matplotlib
		@ INPUT  : None
		@ RETURN : None
	"""	

	def DrawPlt(self):

		########## Calculate explored states
		state_step_explored = sum( [ 1 if i else 0 for i in self.Q_stepTaken.values() ] ) / (( (self.StateSlice+1) ** self.LaunchNums ) * self.ActionNums * OMEGAAUGNUM)
		exp = float(100 * state_step_explored)

		if not ENABLE_ALPHA_DESCEND and not ENABLE_EPSILON_REDUCE:
			self.fig.suptitle("Episode No. %d,  Explored States Rate = %.2f%%,  Avg_steps = %.2f" % (self.Episode, exp, self.avg_step), fontsize = 12)
		elif ENABLE_ALPHA_DESCEND and ENABLE_EPSILON_REDUCE:
			self.fig.suptitle("Ep No.%d,  Explored States Rate = %.2f%%,  Avg_steps = %.2f, epsilon = %.2f, alpha = %.2f" % (self.Episode, exp, self.avg_step, self.EPSILON, self.ALPHA), fontsize = 8)
		elif ENABLE_ALPHA_DESCEND and not ENABLE_EPSILON_REDUCE:
			self.fig.suptitle("Ep No.%d,  Explored States Rate = %.2f%%,  Avg_steps = %.2f, alpha = %.2f" % (self.Episode, exp, self.avg_step, self.ALPHA), fontsize = 10)
		elif not ENABLE_ALPHA_DESCEND and ENABLE_EPSILON_REDUCE:
			self.fig.suptitle("Ep No.%d,  Explored States Rate = %.2f%%,  Avg_steps = %.2f, epsilon = %.2f" % (self.Episode, exp, self.avg_step, self.EPSILON), fontsize = 10)

		ax = plt.subplot("311")
		ax.clear()
		ax.set_title('Steps for Each Episode', fontsize = 8)
		ax.set_ylabel('Steps')
		ax.plot(self.step_saver, 'ko', label='Steps', ms=0.4)
		ax.plot(self.avg_step_saver, 'co-', label='Average', ms=0.5, linewidth=0.4)
		ax.legend(loc="upper left")

		ax2 = plt.subplot("312")
		ax2.clear()
		ax2.set_xlabel('Episode No.')
		ax2.set_ylabel('Rate %')
		ax2.plot(self.max_rate_saver, 'mo-', label = 'Whole Time', ms=.8)
		ax2.plot(self.max_step_per20, 'ro-' , label = 'Last %d Ep' % SHORT_TERM_FOR_MAX_RATE , ms=.8)
		ax2.plot(self.abort_step_saver, 'b*-', label='Aborted Rate', ms=0.5, linewidth=0.4)
		ax2.legend(loc="upper left")
		ax2.set_ylim([-5,105])

		ax3 = plt.subplot("313")
		ax3.clear()
		ax3.set_xlabel('Decisions')
		ax3.set_ylabel('Time (sec.)')
		if len(self.wait_time_saver) > 20000: 
			del self.wait_time_saver[0:len(self.wait_time_saver) - 20000]
		ax3.plot(self.wait_time_saver, 'yo', label='Time Gap', ms=.2)
		ax3.legend(loc="upper left")
		ax3.set_ylim(TIME_GAP_PRINT)

		plt.pause(0.01)



	"""
		@ BRIEF  : Calculate and update Q table based on s, a, r, s'
		@ INPUT  : Current_State, Action, Reward, Next_State( i.e s')
		@ RETURN : New value of Q(s, a) of Q-table
		@ ANNOT  : Note that the argument "state_" is a class member, but still needs to input from wherever it's called.
		         It was meant to maintain the consistency of the ref. document and the code
	"""
	def updateQ(self, current_state, action, reward, state_prime): # done
		### newQ = float(reward) + self.GAMMA * max(Q_value_copy)

		if DISABLE_Q_UPDATE: return float(self.Q[current_state, action])

		Q_value_copy = [ float(self.Q.setdefault((state_prime, i), - 0.01 * float(rand.random()))) for i in range(0, ActionNums) ]
		newQ = (1-self.ALPHA) * float(self.Q[current_state, action]) + self.ALPHA * ( float(reward) + self.GAMMA * max(Q_value_copy))

		##### If the s' remains the same as s and the car isn't crushed, don't update Q-value.
		##### This make sure that higher decision making rate doesn't lower Q-value of current state, s
		if (current_state == state_prime) and reward == 0 : pass   ### Do nothing If State Not Changed
		elif self.wait_time_saver[-1] > TIME_GAP_MAX :             ### Delay TOO LONG! Details In GetCurrentState
			self.abort_this_ep += 1
		else : self.Q[(current_state, action)] = "%f" % newQ
		self.Q_stepTaken[(current_state, action)] = int(self.Q_stepTaken[(current_state, action)]) + 1
		
		return newQ



	"""
		BRIEF  : Record data and print them to the screen
		INPUT  : None
		RETURN : None
	"""
	def record_and_print_data_to_screen(self):

		if self.simulation:

			Gazebo_reset_srv()
			Reset_Model_srv()

		else:

			while True:

				respawn =  raw_input("Press enter when the robot is repositioned...")

				if  respawn == "":
					break


		self.bumper_sensor.reset_bumper_collision()
		
		if self.step <= round(self.avg_step/100) : 
			print(color_R + 'RAPDTS: ' + 'ABORT' + color_W)
			return -1

		self.Episode += 1
		
		########## Plot Data Recording
		self.step_saver.append(self.step)
		self.avg_step_saver.append((self.step-self.avg_step)/(self.Episode)+self.avg_step)
		### Max_Step Rate Calculation
		if self.step >= MAX_STEP :
			if self.first_max == -1 : self.first_max = self.Episode
			self.max_step_times += 1
			self.max_step_recorder.append(1)
		else : self.max_step_recorder.append(0)
		if self.first_max == -1: self.max_rate_saver.append(0)
		else: self.max_rate_saver.append(self.max_step_times/(self.Episode-self.first_max+1)*100)
		### Short Term Performance Calculation
		if (self.Episode) >= SHORT_TERM_FOR_MAX_RATE :
			acc = sum(self.max_step_recorder[-SHORT_TERM_FOR_MAX_RATE:])
			self.max_step_per20.append((acc/SHORT_TERM_FOR_MAX_RATE)*100)
		else : 
			acc = sum(self.max_step_recorder)
			self.max_step_per20.append((acc/self.Episode)*100)
		### Abort Rate Calculation
		self.abort_step_saver.append(self.abort_this_ep/self.step*100)

		########## Avg Performance Calculation
		self.avg_step = (self.step-self.avg_step)/(self.Episode)+self.avg_step
		self.avg_reward = (self.single_episode_reward-self.avg_reward)/(self.Episode)+self.avg_reward
		
		########## Calculate explore rate
		state_step_explored = sum( [ 1 if i else 0 for i in self.Q_stepTaken.values() ] ) / (( (self.StateSlice+1) ** self.LaunchNums ) * self.ActionNums * OMEGAAUGNUM)

		########## Save Q Table And Plot When Specific Episodes are Terminated
		### Functions 'Save_*_At' Defined in ql_adj_func.py
		if Save_Q_Table_At(self.Episode) and not DISABLE_Qvl_SAVING :   
			time_ = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
			dt.WriteData(self, time_ , self.OLD_DATA)
		if Save_Plt_At(self.Episode)     and not DISABLE_PLT_SAVING : 
			cwd = "/home/student/catkin_ws/src/qlearning_project/src/qlearning_project"
			plt.savefig(cwd + FolderName + 'plot/E%d' % (self.Episode) +'.png')
		if Draw_Plt_At(self.Episode) and not DISABLE_PLT_SHOWING : self.DrawPlt()
		
		########## Write Log For every Episodes
		if not DISABLE_LOG_SAVING :
			time_ = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
			dt.WriteLog(self, time_ , self.Episode, self.step, self.single_episode_reward)

		########## Print Result Of The Episode To The Screen
		if   self.step == 1: 
			print '\033[0m',
		elif self.step >= self.avg_step: 
			print '\033[33m',
		elif self.step <  self.avg_step: 
			print '\033[34m',

		print("================================== Episode No. %d Ends ================================" % self.Episode)
		print("================================== Episode Reward = %.3f ===============================" % self.single_episode_reward)
		print("================================== avg Reward     = %d =================================" % self.avg_reward)
		print("================================== total step     = %d ==================================" % self.step)
		print("================================== avg step       = %.2f ===================================" % self.avg_step)
		print("================================== Explored rate  = %.3f%% ===================================" % float(100 * state_step_explored))
		print '\033[0m',

		self.step = self.single_episode_reward = 0
		self.last_dist_cat = []
		self.Q_traj = []
		self.Q_traj_n = 0
		self.abort_this_ep = 0


	################################### ROS operation ###################################

	def Pub_Twist(self, lin, ang):
		if MANUAL: return -1
		self.twist.linear.x  = lin; self.twist.linear.y  = 0 ; self.twist.linear.z  = 0
		self.twist.angular.x = 0  ; self.twist.angular.y = 0 ; self.twist.angular.z = ang
		self.pub.publish(self.twist)
		self.pub_rate.sleep()
		return self.twist

	def cb_scan(self, data):
		ranges = data.ranges
		self.dist=[]
		Slice = int(round (360/self.ScanSlice))
		dist_tmp = [ ranges[i] for i in range(0,360,Slice) ]
		if self.LaunchNums == self.ScanSlice: self.dist = dist_tmp
		elif self.LaunchNums % 2 == 1 :
			d = int((self.LaunchNums-1)/2)
			for i in range( 360-(d*Slice), 360-Slice+1, Slice ): self.dist.append(ranges[i])
			for i in range( 0, (d+1) * Slice, Slice ): self.dist.append(ranges[i])

	def cb_odom(self, data):
		# rospy.sleep(SLEEP_TIME/5)
		self.omega = float(format( data.twist.twist.angular.z, '0.1f' ))



################################### End Of class QLbot ###################################



def Gazebo_reset_srv():
	if DISABLE_GAZEBO_RESET : return None
	srv_reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
	rospy.wait_for_service('/gazebo/reset_world')
	print("call rosservice: /gazebo/reset_world")
	
	try:
		srv_reset_world()
	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)

def Reset_Model_srv():
	if DISABLE_MODEL_RESET : return None
	srv_reset_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	rospy.wait_for_service('/gazebo/set_model_state')
	print("call rosservice: /gazebo/set_model_state")
	model_state = ModelState()
	model_state.model_name = 'mobile_base'
	Set_Model(model_state, -1)
	try: 
		srv_reset_model_state(model_state)
		rospy.sleep(0.2)
	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)


### For Debug
if __name__ == '__main__':

	print("qlbot.py start")
	cwd = os.getcwd()
	a = QLbot('2017-08-17_10-55-17.csv')
	
	dt.WriteData(a, 'qqq', a.OLD_DATA)

	print("qlbot.py end")

	pass

