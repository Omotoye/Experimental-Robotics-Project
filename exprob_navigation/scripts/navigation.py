#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
# import actionlib  # would be needed for later versions 

# the robot nav service messages  
from exprob_msgs.srv import RobotNav, RobotNavResponse

# for wasting time 
import time
import random  


def go_to_poi(poi_req):
    # get the coordinate corresponding to the point of interest given 
    goal_cord = rospy.get_param(f'/poi_map_cord/{poi_req.goal}')
    # the goal cord would be used in later versions 

    print(f'Robot Navigating to the {goal_cord.loc_name} at Coordinates x: {goal_cord.x}, y: {goal_cord.y}')

    # waste time to simulate motion 
    time.sleep((3 * random.random()))

    # return the result of executing the action
    return 'goal reached'  


def handle_robot_nav(req):
    # Checking if the Point of Interest exists
    if (rospy.has_param(f'/poi_map_cord/{req.goal}')):
        result = go_to_poi(req.goal)
        return RobotNavResponse(result)
    else:
        return RobotNavResponse('Invalid POI')


def main():
    # Initialize the ros node 
    rospy.init_node('robot_navigation')
    
    # Initialize the service 
    rospy.Service('robot_nav_srv', RobotNav, handle_robot_nav)
    
    # Keeps the node alive to listen for client requests
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass  
