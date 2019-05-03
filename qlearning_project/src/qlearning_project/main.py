#!/usr/bin/env python
from datetime import datetime
from qlbot import *
import record_data as dt
import time

x = None

def main():

	#Initializing ros main node
	rospy.init_node("qlbot", anonymous = True)

	global x

	x = QLbot(LoadName, True)

	if not x.start :
		return None

	while not rospy.is_shutdown():
		
		isEnd = None

		while x.state != (-1, -1) :
			
			print("------------------------- Episode: %d -------------------------" % int(x.Episode+1))

			if isEnd is True or isEnd is None : x.state = x.GetCurrentState()
			else: x.state = state_p

			action, action_policy = x.ChooseAction(x.state)

			isEnd, state_p, reward = x.MoveToNextStep(action, action_policy)

			print( "Main: ", x.state, state_p )

			x.updateQ(x.state, action, reward, state_p)

if __name__ == '__main__':
	main()	