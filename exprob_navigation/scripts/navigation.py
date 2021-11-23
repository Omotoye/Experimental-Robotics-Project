#!/usr/bin/env python

"""
.. module:: navigation
    :platform: Unix
    :synopsis: Python module for control of a mobile robot to navigate to a target point
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 
This node is supposed, controls a mobile robot to move from it position to some target position
but this functionality would be implemented in later versions, for now it just waste time. 
    
Service:
    /robot_nav_srv accept a request of the id of the target point of interest 
    
"""

import rospy

# Brings in the SimpleActionClient
# import actionlib  # would be needed for later versions

# the robot nav service messages
from exprob_msgs.srv import RobotNav, RobotNavResponse

# for wasting time
import time
import random


def go_to_poi(poi_req):
    """This fuction simulates motion in the environment by simply
    wasting time.

    Args:
        poi_req (str): The name of the location the robot is supposed to
        navigate to

    Returns:
        str: a string return when the simulated goal is reached.
    """
    # get the coordinate corresponding to the point of interest given
    goal_cord = rospy.get_param(f"/map/{poi_req}")
    # the goal cord would be used in later versions

    name = goal_cord["loc_name"]

    rospy.loginfo(
        f"Robot Navigating to the {goal_cord['loc_name']} at Coordinates x: {goal_cord['x']}, y: {goal_cord['y']}"
    )

    # waste time to simulate motion
    time.sleep((5 * random.random()))

    # return the result of executing the action
    return "goal reached"


def handle_robot_nav(req):
    # Checking if the Point of Interest exists
    if rospy.has_param(f"/map/{req.goal}"):
        result = go_to_poi(req.goal)
        return RobotNavResponse(result)
    else:
        return RobotNavResponse("Invalid POI")


def main():
    # Initialize the ros node
    rospy.init_node("robot_navigation")

    # Initialize the service
    rospy.Service("robot_nav_srv", RobotNav, handle_robot_nav)

    # Keeps the node alive to listen for client requests
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
