#!/usr/bin/env python

import rospy

# the cluedo hint service messages  
from exprob_msgs.srv import CluedoHint, CluedoHintResponse

# for generating random hint
import random  


class HintManager: 
	
	def __init__(self): 
		self.possible_hint_ids = []
		self._get_all_ids()

		# Initializing the hint service 
		rospy.Service("/cluedo_hint", CluedoHint, self.hint_clbk)


	def hint_clbk(self, msg): 
		response = CluedoHintResponse()
		hint_id = self._generate_rand_hint_id()
		if hint_id != -1:

			response.hint_id = hint_id
			response.result = "ok"
		else:
			response.result = "no new hint"
		return response 


	def _get_all_ids(self):
		# This function generates all the possible id's for getting
		# data from the hint parameter server. 
		hypo = rospy.get_param('/hints')
		for i in range(len(hypo)):
			for key in hypo[i]:
				for j in range(len(hypo[i][key])):
					self.possible_hint_ids.append(str(i)+str(j))


	def _generate_rand_hint_id(self):
		if len(self.possible_hint_ids) > 0:
			rand_id = random.randint(0,(len(self.possible_hint_ids)-1))
			return self.possible_hint_ids.pop(rand_id)
		else:
			return -1
			 

if __name__ == '__main__':
	rospy.init_node('hint_manager')
	HintManager()
	rospy.spin()

