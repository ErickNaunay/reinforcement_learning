#!/usr/bin/env python

################################# Program Options #######################################
### Program Options ( default: False )
MANUAL = False                           # Control Turtlebot Using teleop  if True
DISABLE_Q_UPDATE     = False             # Won't Modify Q Valie            if True
DISABLE_GAZEBO_RESET = False             # Won't Reset After Car Crushing  if True
DISABLE_MODEL_RESET  = False             # Won't Reset After Car Crushing  if True 
DISBLE_SET_MODEL     = True 
DISABLE_SEARCHING_WITH_SPIN = True 
MAX_STEP = 2000                          # Steps to Terminate Episode when Reached


################################# Data I/O ###############################################
### Data folder
FolderName = '/data/data08235/'

### Data Recorfing ( default: False )
DISABLE_PLT_SHOWING = False             # Won't Show Plots on Screen      if True
DISABLE_PLT_SAVING  = False             # Won't save plt                  if True 
DISABLE_LOG_SAVING  = False             # Won't save data of log          if True 
DISABLE_Qvl_SAVING  = False             # Won't save data of Q-table      if True

SHORT_TERM_FOR_MAX_RATE = 40

### Data Reading
# Load Q Table And Params From This File from directory LOG
# If LoadName is None, Than Other Params Will be Ignored
LoadName = None # '2017-09-02_19-16-44.csv'
#LoadName= "2019-04-24_21-19-52.csv"
RECORD_PREVIOUS_PARAM = True             # Printing On Header of New File  if True
USE_PREVIOUS_PARAMETERS = UPP = True     # Refer to USE_DATA
USE_DATA = { 'Episode':False, 'avg_step':False, 'avg_reward':False, 
				'ALPHA':True, 'GAMMA':True, 'EPSILON':True, 'SLEEP_TIME':True,
				'angularSpeed':True, 'linearSpeed':True, 'ActionNums':True,
				'StateSlice': True, 'SensingThreshold__':True,
				'ScanSlice':True, 'LaunchNums':True 
				,'Q':True, 'Q_stepTaken':True }




################################# PARAMs TUNNING #########################################

### Used in ChooseAction
### Decide Initial Value, RANDOM_Q * float(rand.random()), For Unexplored Q
RANDOM_Q = 0

### Angular Speed As States
OMEGAAUG = True  ### Only avail for recording now, NEEDS TO IMPROVE
OMEGAAUGNUM = 3

### Learning Factors 
# newQ = (1-ALPHA) * float(self.Q[current_state, action]) + ALPHA * ( float(reward) + GAMMA * max(Q_value_copy))
GAMMA = 0.58      # Discount Factor
ALPHA = 0.42      # Learning Rate
ENABLE_ALPHA_DESCEND  = False
ALPHA_DESCEND = 0.95
ENABLE_EPSILON_REDUCE = True
EPSILON = 0.32   # For Epsilon-Greedy

# In ChooseAction()
# self.EPSILON = reduce_epsilon(self.Episode, EPSILON)
def reduce_epsilon(episode, now):
	if not ENABLE_EPSILON_REDUCE: return now
	if episode == 400: return 0.12
	if episode == 800: return 0.08
	if episode == 1600: return 0.04
	if episode == 2000: return 0.01
	if episode == 5000: return 0.001
	else: return now

### Execute Time Gap
SLEEP_TIME = 0.05
TIME_GAP = 0.1
TIME_GAP_WAIT = TIME_GAP/20
TIME_GAP_MAX_DIFF = TIME_GAP/5
TIME_GAP_MAX = TIME_GAP + TIME_GAP_MAX_DIFF
TIME_GAP_PRINT = [0.08, 0.16]
TIME_GAP_ABORT = 0.15

### Turtlebot Parameters
angularSpeed = 1.28
linearSpeed = 0.32
ActionNums = 9

### Sensor State Parameters
# State Numbers = (StateSlice+1)**LaunchNums
StateSlice = 3
SensingThreshold__ = [0.26, 0.72, 1.2] #[0.36, 0.84, 1.4] 
ScanSlice = 12
LaunchNums = 3

# In moveToNextStep( self, action, action_policy )
# elif ep_terminate(bump, obstacles, self.dist, LaunchNums, StateSlice):
def ep_terminate(bump, obs, dist_cat, LN, StS): ### Decided Case by Case
	if LN == 3 and StS == 2:
		if bump >= 2 or (bump == 1 and obs[StS-2] == 2):
			return True
		else: return False
	elif LN == 5 and StS == 3:
		if bump >= 2 or (bump + obs[1] >= LN-2 ) : return True
		else : return False
	elif LN == 5 and StS == 4:
		if bump >= 3 : return True
		else : return False
	elif LN == 3 and StS == 3:
		if bump >= 2 or bump+obs[1]>=LN :
			return True
		else : return False
	else:
		### Only In General, Still Needs To Decide By Case
		if bump > int(LN/2) or (bump + obs[StS-2] >= LN-1):
			return True
		else: return False

################################# Display Color Code #####################################
color_W = '\033[0m'
color_R = '\033[31m'
color_G = '\033[32m'
color_BG = '\033[1;32m'
color_O = '\033[33m'
color_Y = '\033[1;33m'
color_B = '\033[34m'
color_S = '\033[1;34m'
color_P = '\033[35m'
color_BP = '\033[1;35m'

################################# main() For Debug #######################################
if __name__ == '__main__':
	if (ScanSlice < LaunchNums) :
		print(color_R+"Error: ScanSlice < LaunchNums"+color_W)
	elif ScanSlice!=LaunchNums:
		print(color_O+"Warning: ScanSlice != LaunchNums"+color_W)
	else : print(color_B+"Seems No Setting Errors in QLbotConfig.py"+color_W)
	pass
