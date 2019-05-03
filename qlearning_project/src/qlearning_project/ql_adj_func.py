from qlbotconf import *

################################# Func. For Quick Adjust #################################


# In record_and_print_data_to_screen(self)
# if Save_Q_Table_At(self.Episode)
def Save_Q_Table_At(episode):
	if DISABLE_Qvl_SAVING: return False
	save_episode=[1, 10, 25, 50, 100, 150, 300]
	save_per = 250
	if (episode in save_episode) or episode % save_per == 0: 
		return True
	else: return False

# In record_and_print_data_to_screen(self)
# if Save_Plt_At(self.Episode)
def Save_Plt_At(episode):
	if DISABLE_PLT_SAVING: return False
	save_episode=[1, 25, 50, 75, 100, 150, 200, 300, 400]
	save_per = 250
	if (episode in save_episode) or episode % save_per == 0: 
		return True
	else: return False

# In record_and_print_data_to_screen(self)
# if Draw_Plt_At(self.Episode)
def Draw_Plt_At(episode):
	if DISABLE_PLT_SHOWING: return False
	draw_episode=[1, 5, 10, 15, 20]
	draw_per = 50
	if (episode-1 in draw_episode) or (episode-1) % draw_per == 0: 
		return True
	else: return False


# In GetCurrentState()
# current_om_state = Cal_OM_State(self.omega, self.angularSpeed)
def Cal_OM_State(omega, angularSpeed):
	if   omega < - angularSpeed/8  : return 0
	elif omega >   angularSpeed/8  : return 2
	else : return 1

# In moveToNextStep( self, action, action_policy )
# if is_bad_state(self): DeadEnd = True
def is_bad_state(self_):
	AN = self_.ActionNums
	Q   = [float(self_.Q[(self_.state, i)]) for i in range (0, self_.ActionNums)]
	QST = [float(self_.Q_stepTaken[(self_.state, i)]) for i in range (0, self_.ActionNums)]
	if risk_evaluator(self_, Q)[1] <= 0 and all(Q[i]<=ALT for i in range(0, AN)) and all(QST[i]>=ABT for i in range(0,AN)) : return True 
	else : return False

# In MoveToNextStep : risk_now, risk_new
# In is_bad_state   : if risk_evaluator(self_, Q) <= -4 ... : return True
def risk_evaluator(self_, dist_cat):
	StS = self_.StateSlice
	LN  = self_.LaunchNums
	if StS == 3:
		countDC = [dist_cat.count(i) for i in range(0,StS)] 
		countDC.append(dist_cat.count(-1))
		Rank = 0
		if   countDC[0] == LN                                           : Rank = 3
		elif countDC[0] + countDC[1] == LN and countDC[0] >  countDC[1] : Rank =  2
		elif countDC[0] + countDC[1] == LN and countDC[0] <= countDC[1] : Rank =  1
		elif countDC[StS] >  1                                          : Rank = -6
		elif countDC[StS] == 1 and countDC[StS-1] >= 1                  : Rank = -5
		elif countDC[StS] == 1 and countDC[StS-1] == 0                  : Rank = -4
		elif countDC[StS] == 0 and countDC[StS-1] == LN                 : Rank = -3
		elif countDC[StS] == 0 and countDC[StS-1] == LN-1               : Rank = -2
		elif countDC[StS] == 0 and countDC[StS-1] != 0                  : Rank = -1
		else : Rank = 0

		if Rank >= 0 : return 'S', Rank         # Safe
		else : return 'NS', Rank                # Not Safe

# In MoveToNextStep : reward = reward_giver(risk_now, risk_new, snow, snew)
def reward_giver(rnow, rnew, snow, snew):
	if snow is snew : return 0     ###################################### Don't Give Any Reward If State Are Not Changed
	elif rnow[0] == rnew[0]:       ###################################### Same Condition S-S or NS-NS
		if rnew[0] == 'S':         ######################  S to  S
			if   rnew[1] >  rnow[1] : return  0.4
			elif rnew[1] == rnow[1] : return  0.1
			elif rnew[1] <  rnow[1] : return -0.12
		elif rnew[0] == 'NS':      ###################### NS to NS
			if   rnew[1] >  rnow[1] : return  0.15
			elif rnew[1] == rnow[1] : return -0.2
			elif rnew[1] <  rnow[1] : return -0.4
	else:                          ###################################### Different Condition NS-S or S-NS
		if rnew[1] > rnow[1]        : return  0.8    ########### NS to S
		else                        : return -0.8   ###########  S to NS

def Set_Model(model_state, n):

	if DISBLE_SET_MODEL:
		return None

	from math import pi
	from random import randrange

	linear = model_state.twist.linear
	angular = model_state.twist.angular
	position = model_state.pose.position
	orientation = model_state.pose.orientation
	linear.x, linear.y, linear.z = (0,0,0)
	angular.x, angular.y, angular.z = (0,0,0)

	pos_num = 11
	pos_set = {
			0:((-2,-0.5,0),(0,0,0,0)), 
			1:((1.25,-0.5,0.0),(0,0,0,0)), 
			2:((0.8,0.8,0),(0,0,1,4.2)), 
			3:((-0.8,0,0),(0,0,0,0)), 
			4:((0, 2, 0),(0,0,1,-pi/4)), 
			5:((0,1.52,0),(0,0,1,pi/4)),
			6:((-2.2,0,0),(0,0,0,0)),
			7:((-1.5,1.45,0),(0,0,1,pi/2)),
			8:((-1.5,-1.45,0),(0,0,1,-pi/2)),
			9:((-1,1,0),(0,0,1,-pi)),
			10:((1.6,-0.5,0),(0,0,0,0))
		}

	if n == -1 or n not in range(0, pos_num):
		p, o = pos_set[randrange(0, pos_num)]
		position.x, position.y, position.z = p
		orientation.x, orientation.y, orientation.z, orientation.w = o
	else:
		p, o = pos_set[n]
		position.x, position.y, position.z = p
		orientation.x, orientation.y, orientation.z, orientation.w = o

