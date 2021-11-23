"""
.. module:: states
    :platform: Unix
    :synopsis: Python module for initializing all the states of the Cluedo State Machine
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 
This node initializes the states of each of the goals that robot object is required to 
perform for the Cluedo Game Robot Senario
    
Action:
    /robot_object Sends request of each of the required goals to be performed at each 
    state.  
    
"""

import rospy

# importing the library for the creation of the state machine
import smach

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus

# Here we bring in all the messages required to interface with each of the nodes
from exprob_msgs.msg import RobotGoal, RobotResult, RobotAction


def call_robot_action(goal_req):
    """This is a function for calling the robot object action server
    to perform each of the actions required for a state

    Args:
        goal_req ([RobotGoal]): The object containing the goal message for the
        robot action server.

    Returns:
        [RobotResult]: the response message from the action server
    """

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient("robot_object", RobotAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Sends the goal to the action server.
    client.send_goal(goal_req)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # return the result of executing the action
    return client.get_result()


###+++++++++++++++++++ GoTo Room +++++++++++++++++++++++++###
class GoToRoom(smach.State):
    """This is the state class for navigating the robot to a room randomly

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["at room", "not at room"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "go to room"
        result = call_robot_action(robot_goal)
        if result.result == "goal reached":
            rospy.loginfo(f"The Robot reached the goal position")
            return "at room"
        else:
            rospy.loginfo(f"The Robot failed to reach the goal position")
            return "not at room"


###+++++++++++++++++++ GoTo Oracle +++++++++++++++++++++++++###
class GoToOracle(smach.State):
    """This is the state class for navigating the robot to the oracle location
    after finding a correct and complete hypothesis

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["reached oracle", "failed to reach oracle"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "go to oracle"
        result = call_robot_action(robot_goal)
        if result.result == "goal reached":
            rospy.loginfo(f"The Robot reached the Oracle position")
            return "reached oracle"
        else:
            rospy.loginfo(f"The Robot failed to reach the Oracle position")
            return "failed to reach oracle"


###+++++++++++++++++++ Search Hint +++++++++++++++++++++++++###
class SearchHint(smach.State):
    """This is the state class for call the action for search for hint in
    a room

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["no hint", "found hint"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "search hint"
        result = call_robot_action(robot_goal)

        if result.result != "no hint":
            rospy.loginfo(f"Hint with ID {result.result} was found")
            return "found hint"
        else:
            rospy.loginfo(f"No hint was found")
            return "no hint"


###+++++++++++++++++++ Check Hypothesis +++++++++++++++++++++++++###
class CheckHypothesis(smach.State):
    """This is the state class for asking for ontology for a complete and consistent
    hypothesis

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["complete & consistent hypo found", "not found", "check failed"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "hypo check"
        result = call_robot_action(robot_goal)

        if result.result == "hypo found":
            rospy.loginfo("A new Complete and Consistent hypothesis has been found")
            return "complete & consistent hypo found"
        elif result.result == "not found":
            rospy.loginfo("No new Complete and Consistent hypothesis was found")
            return "not found"
        else:
            rospy.loginfo("The Ontology query failed")
            return "check failed"


###+++++++++++++++++++ Update Knowledge +++++++++++++++++++++++++###
class UpdateKnowledge(smach.State):
    """This is the state for updating the cluedo ontology with the new hint
    that has just been found

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["knowledge updated", "knowledge update failed"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "update"
        result = call_robot_action(robot_goal)

        if result.result == "updated":
            rospy.loginfo("The Cluedo Ontology has been updated with the new hint")
            return "knowledge updated"
        else:
            rospy.loginfo("The Ontology update failed")
            return "knowledge update failed"


###+++++++++++++++++++ Oracle (Hypothesis Check) +++++++++++++++++++++++++###
class OracleCheck(smach.State):
    """This is the state class for asking the oracle if the found hypothesis is the
    correct one.

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["hypothesis correct", "hypothesis wrong", "oracle check failed"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "oracle check"
        result = call_robot_action(robot_goal)

        if result.result == "correct hypothesis":
            rospy.loginfo(
                "Congratulations Player!, you've just figured out the correct hypothesis"
            )
            return "hypothesis correct"

        elif result.result == "wrong hypothesis":
            rospy.loginfo(
                "Wrong!, keep getting hints, maybe you'll eventually figure out the right hypothesis :)"
            )
            return "hypothesis wrong"
        else:
            rospy.loginfo("The Oracle check has failed")
            return "oracle check failed"


###+++++++++++++++++++ Announce Hypothesis +++++++++++++++++++++++++###
class AnnounceHypothesis(smach.State):
    """This is the state class for announcing the found hypothesis in
    english language

    Args:
        smach ([object]): The smach class for initializing the state class
    """

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["hypothesis announced", "failed to announce"],
            # output_keys=[],
            # input_keys=[],
        )

    def execute(self, userdata):
        robot_goal = RobotGoal()
        robot_goal.goal = "announce hypo"
        result = call_robot_action(robot_goal)

        if result.result == "announced":
            rospy.loginfo(
                "The Robot has announced a complete and Consistent hypothesis"
            )
            return "hypothesis announced"

        else:
            rospy.loginfo("The hypothesis announcement has failed")
            return "failed to announce"
