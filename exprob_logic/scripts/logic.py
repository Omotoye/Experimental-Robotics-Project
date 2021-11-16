#!/usr/bin/env python

import rospy

# importing the library for the creation of the state machine
import smach
import smach_ros 

# Importing the states for the state machines
from modules.states import *

if __name__ == "__main__":
    rospy.init_node("cluedo_state_machine")
    

    # Create a SMACH state machine
    Cluedo = smach.StateMachine(outcomes=["Game Won!!!"])

    # Open the State Machine Container
    with Cluedo:
    	# Add states to the base container
        smach.StateMachine.add(
                "GoTo Room",
                GoToRoom(),
                transitions={
                    "at room": "Knowledge Management",
                    "not at room": "GoTo Room",
                },
                # remapping={"": ""},
            )
            
            
        KnowledgeManagement = smach.StateMachine(outcomes=["good hypothesis found","no good hypothesis found" ])

        # Open the sub state machine container 
        with KnowledgeManagement: 

            # Add states to the sub state machine container 
            smach.StateMachine.add(
                "Search Hint",
                SearchHint(),
                transitions={
                    "no hint": "Search Hint", "found hint": "Update Knowledge",
                },
                # remapping={"": ""},
            )

            smach.StateMachine.add(
                "Update Knowledge", 
                UpdateKnowledge(), 
                transitions={
                    "knowledge updated": "Check for Consistent & Complete Hypothesis", 
                    "knowledge update failed": "Update Knowledge",
                })

            # Add states to the sub state machine container 
            smach.StateMachine.add(
                "Check for Consistent & Complete Hypothesis",
                CheckHypothesis(),
                transitions={
                    "complete & consistent hypo found": "good hypothesis found",
                    "not found" : "no good hypothesis found",
                    "check failed": "Check for Consistent & Complete Hypothesis"
                },
                # remapping={"": ""},
            )

        # Add sub state machine to the base state machine container 
        smach.StateMachine.add(
            "Knowledge Management", KnowledgeManagement, transitions={"no good hypothesis found": "GoTo Room", "good hypothesis found":"GoTo Oracle"}
        )

        # Add states to the base container
        smach.StateMachine.add(
                "GoTo Oracle",
                GoToOracle(),
                transitions={
                    "reached oracle": "Announce Hypothesis",
                    "failed to reach oracle": "GoTo Oracle"
                },
                # remapping={"": ""},
            )

        # Add states to the base container
        smach.StateMachine.add(
                "Announce Hypothesis",
                AnnounceHypothesis(),
                transitions={
                    "hypothesis announced": "Oracle (Hypothesis Check)",
                    "failed to announce": "Announce Hypothesis"
                },
                # remapping={"": ""},
            )

        # Add states to the base container
        smach.StateMachine.add(
                "Oracle (Hypothesis Check)",
                OracleCheck(),
                transitions={
                    "hypothesis correct": "Game Won!!!",
                    "hypothesis wrong": "GoTo Room",
                    "oracle check failed": "Oracle (Hypothesis Check)"
                },
                # remapping={"": ""},
            )

            


        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer(
            "server_name", Cluedo, "Cluedo Game Robotics Scenario"
        )
        sis.start()

        # Execute SMACH plan
        outcome = Cluedo.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()
