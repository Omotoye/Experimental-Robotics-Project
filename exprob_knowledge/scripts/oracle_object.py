#!/usr/bin/env python

"""
.. module:: oracle_object
    :platform: Unix
    :synopsis: Python module for generating hint and confirming hypothesis
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 
This node generate a random hint from the cluedo hint parameter server and also 
determines the hypothesis that wins the game. 
    
Service:
    /oracle_srv accepts a request to generate hint and check hypothesis.  
    
"""

import rospy

# the oracle service messages
from exprob_msgs.srv import Oracle, OracleResponse

# for generating random hint
import random


class GameOracle:
    """This is the Oracle class that takes care of generating hint and
    determining the right hypothesis
    """

    def __init__(self):
        self.possible_hint_ids = []
        self.hypo = rospy.get_param("/hints")
        self._get_all_ids()
        self.true_hypothesis = self._generate_true_hypothesis()
        rospy.loginfo(f"\n\nThe True hypothesis ID is {self.true_hypothesis}\n\n")

        # Initializing the hint service
        rospy.Service("/oracle_srv", Oracle, self.oracle_clbk)

    def oracle_clbk(self, msg):
        response = OracleResponse()

        if msg.goal == "generate hint":
            hint_id = self._generate_rand_hint_id()
            if hint_id != -1:
                response.hint_id = hint_id
                response.result = "ok"
            else:
                response.result = "no new hint"
            return response
        else:
            # Check Hypothesis
            if msg.hypo_id == self.true_hypothesis:
                response.result = "correct hypothesis"
            else:
                response.result = "wrong hypothesis"
            return response

    def _get_all_ids(self):
        # This function generates all the possible id's for getting
        # data from the hint parameter server.
        for i in range(len(self.hypo)):
            for key in self.hypo[i]:
                for j in range(len(self.hypo[i][key])):
                    self.possible_hint_ids.append(str(i) + str(j))

    def _generate_rand_hint_id(self):
        if len(self.possible_hint_ids) > 0:
            rand_id = random.randint(0, (len(self.possible_hint_ids) - 1))
            return self.possible_hint_ids.pop(rand_id)
        else:
            return -1

    def _generate_true_hypothesis(self):
        test_list = []
        required_list = ["who", "where", "what"]
        correct_hypo = []

        for i in range(len(self.hypo)):
            for key in self.hypo[i]:
                for j in range(len(self.hypo[i][key])):
                    keys = list(self.hypo[i][key][j])
                    test_list.append(keys[0])
                    required_list.sort()
                    test_list.sort()
            if test_list == required_list:
                correct_hypo.append(str(i))
            test_list = []
        return correct_hypo[random.randint(0, (len(correct_hypo) - 1))]


if __name__ == "__main__":
    rospy.init_node("oracle_node")
    GameOracle()
    rospy.spin()
