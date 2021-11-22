#! /usr/bin/env python

import rospy

import actionlib

from exprob_msgs.msg import RobotFeedback, RobotResult, RobotAction
from exprob_msgs.srv import Knowledge, Oracle, RobotNav
from exprob_msgs.srv import KnowledgeRequest, OracleRequest, RobotNavRequest

from Camera import Camera

import random


class Robot(object):
    def __init__(self, name):
        self._action_name = name
        # create messages that are used to publish feedback/result
        self._feedback = RobotFeedback()
        self._result = RobotResult()
        self._possible_loc = [
            "lounge",
            "dinning_room",
            "kitchen",
            "hall",
            "study",
            "library",
            "billiard_room",
            "conservatory",
            "ballroom",
        ]
        self._oracle_pose_id = "oracle_room"
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            RobotAction,
            execute_cb=self.robot_action_cb,
            auto_start=False,
        )
        self._as.start()
        self.rate = rospy.Rate(1)
        self.current_hint_id = None
        self.checked_hypo = []
        self.new_hypo = None
        self.status = None

    def robot_action_cb(self, goal):
        if goal.goal == "search hint":
            result = self.consult_oracle(goal.goal)
            if result != -1:
                self._result.result = result
            else:
                self._result.result = "no hint"

        elif goal.goal == "update":
            result = self.call_knowledge(goal.goal)
            if result.result:
                self._result.result = result.result
            else:
                self._result.result = "update failed"

        elif goal.goal == "hypo check":
            result = self.call_knowledge(goal.goal)
            if result.result == "hypo found":
                for item in result.hypo_ids:
                    if item not in self.checked_hypo:
                        self.new_hypo = item
                if self.new_hypo:
                    self._result.result = result.result
                    self.status = f"Found {self.new_hypo}"
                    self.publish_feedback()
                else:
                    self._result.result = "not found"
            else:
                self._result.result = "not found"

        elif goal.goal == "go to room":
            result = self.go_to_poi(goal.goal)
            if result.result:
                self._result.result = result.result
            else:
                self._result.result = "navigation failed"

        elif goal.goal == "go to oracle":
            result = self.go_to_poi(goal.goal)
            if result.result:
                self._result.result = result.result

        elif goal.goal == "announce hypo":
            result = self.call_knowledge(goal.goal)
            if result.result:
                self._result.result = result.result

        elif goal.goal == "oracle check":
            result = self.consult_oracle(goal.goal)
            if result.result:
                self._result.result = result.result
            else:
                self._result.result = "oracle check failed"
        if self._result.result:
            self.publish_result()

    def go_to_poi(self, goal):
        req = RobotNavRequest()
        if goal == "go to room":
            rand_pose = self.get_rand_pose()
            req.goal = rand_pose

        elif goal == "go to oracle":
            req.goal = self._oracle_pose_id
        self.status = "calling the navigation service...."
        self.publish_feedback()
        response = self.call_service(
            req=req, srv_name="robot_nav_srv", srv_type=RobotNav()
        )
        self.status = response.result
        self.publish_feedback()
        return response

    def consult_oracle(self, goal):
        if goal == "search hint":
            camera_object = Camera()
            self.status = "The camera is active and will start looking for hint"
            self.publish_feedback()
            result = camera_object.get_hint()
            if result != -1:
                self.status = f"A hint of with hind_id: {result} has been found"
                self.publish_feedback()
                self.current_hint_id = result
            else:
                self.status = f"There was a problem with finding hint"
                self.publish_feedback()
            return result

        elif goal == "oracle check":
            req = OracleRequest()
            req.goal = goal
            req.hypo_id = self.new_hypo
            response = self.call_service(
                req=req, srv_name="/oracle_srv", srv_type=Oracle()
            )
            self.checked_hypo.append(self.new_hypo)
            self.new_hypo = None
            return response

    def call_knowledge(self, goal):
        req = KnowledgeRequest()
        if goal == "hypo check":
            req.goal = goal

        elif goal == "update":
            req.goal = goal
            req.hint_id = self.current_hint_id

        elif goal == "announce hypo":
            req.goal = goal
            req.hypo_id = self.new_hypo
        self.status = "calling the knowledge manager service...."
        self.publish_feedback()
        response = self.call_service(
            req=req, srv_name="/knowledge_srv", srv_type=Knowledge()
        )
        self.status = "gotten response from the knowledge manager"
        self.publish_feedback()
        return response

    def call_service(self, req, srv_name, srv_type):
        rospy.wait_for_service(srv_name)
        try:
            response = rospy.ServiceProxy(srv_name, srv_type)
            return response(req)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def publish_result(self):
        rospy.loginfo("%s: Succeeded" % self._action_name)
        self._as.set_succeeded(self._result)

    def check_prempt_request(self):
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self._action_name)
            self._as.set_preempted()

    def publish_feedback(self):
        self.check_prempt_request()
        self._feedback.task_state = self.status
        # publish the feedback
        self._as.publish_feedback(self._feedback)
        # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        self.rate.sleep()

    def get_rand_pose(self):
        rand_index = random.randint(0, len(self._possible_loc) - 1)
        return self._possible_loc[rand_index]


if __name__ == "__main__":
    rospy.init_node("robot_object")
    Robot(rospy.get_name())
    rospy.spin()
