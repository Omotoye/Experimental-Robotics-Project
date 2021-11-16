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
            self,  outcomes=["at room", "not at room"], 
            # output_keys=[],
            # input_keys=[],
        )
    def execute(self, userdata):
        if True:
            return "at room"
        else:
            return "not at room"


###+++++++++++++++++++ GoTo Oracle +++++++++++++++++++++++++###
class GoToOracle(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["reached oracle", "failed to reach oracle"], 
            # output_keys=[],
            # input_keys=[],
        )
    def execute(self, userdata):
        if True: 
            return "reached oracle"
        else: 
            return "failed to reach oracle"


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


###+++++++++++++++++++ Check Hypothesis +++++++++++++++++++++++++###
class CheckHypothesis(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["complete & consistent hypo found","not found", "check failed"], 
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        x = False 
        if True: 
            return "complete & consistent hypo found" 
        elif x: 
            return "check failed"
        else: 
            return "not found"


###+++++++++++++++++++ Update Knowledge +++++++++++++++++++++++++###
class UpdateKnowledge(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["knowledge updated","knowledge update failed"], 
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        if True: 
            return "knowledge updated" 
        else: 
            return "knowledge update failed"


###+++++++++++++++++++ Oracle (Hypothesis Check) +++++++++++++++++++++++++###
class OracleCheck(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["hypothesis correct","hypothesis wrong", "oracle check failed"], 
            # output_keys=[],
            # input_keys=[],
        )
    
    def execute(self, userdata):
        x = False
        if True: 
            return "hypothesis correct" 
        elif x: 
            return "oracle check failed"
        else: 
            return "hypothesis wrong"


###+++++++++++++++++++ Announce Hypothesis +++++++++++++++++++++++++###
class AnnounceHypothesis(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,  outcomes=["hypothesis announced", "failed to announce"], 
            # output_keys=[],
            # input_keys=[],
        )
    
    def execute(self, userdata):
        if True: 
            return "hypothesis announced"
        else: 
            return "failed to announce"





