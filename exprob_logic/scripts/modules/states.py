import rospy

# importing the library for the creation of the state machine
import smach


# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus

# Here we bring in all the messages required to interface with each of the nodes
## TODO: Import all the required messages for services and actions. 

import time


###+++++++++++++++++++ GoTo Room +++++++++++++++++++++++++###
class GoToRoom(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["at room"], 
            # output_keys=[],
            # input_keys=[],
        )
    def execute(self, userdata):
        if True:
            return "at room"


###+++++++++++++++++++ GoTo Oracle +++++++++++++++++++++++++###
class GoToOracle(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["reached oracle"], 
            # output_keys=[],
            # input_keys=[],
        )
    def execute(self, userdata):
        if True: 
            return "reached oracle"


###+++++++++++++++++++ Search Hint +++++++++++++++++++++++++###
class SearchHint(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["no hint", "found hint"], 
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        if True: 
            return "found hint"
        else: 
            return "no hint" 


###+++++++++++++++++++ Check Hint Consistency +++++++++++++++++++++++++###
class CheckHintConsistency(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["consistent","inconsistent"], 
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        if True: 
            return "consistent" 
        else: 
            return "inconsistent"


###+++++++++++++++++++ Oracle (Hypothesis Check) +++++++++++++++++++++++++###
class OracleCheck(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["hypothesis correct","hypothesis wrong"], 
            # output_keys=[],
            # input_keys=[],
        )
    
    def execute(self, userdata):
        if True: 
            return "hypothesis correct" 
        else: 
            return "hypothesis wrong"


###+++++++++++++++++++ Announce Hypothesis +++++++++++++++++++++++++###
class AnnounceHypothesis(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["hypothesis announced"], 
            # output_keys=[],
            # input_keys=[],
        )
    
    def execute(self, userdata):
        if True: 
            return "hypothesis announced"





