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
                    "at room": "Get Hint",
                },
                # remapping={"": ""},
            )
            
            
        GetHint = smach.StateMachine(outcomes=["hint not found", "good hint", "bad hint"])

        # Open the sub state machine container 
        with GetHint: 

            # Add states to the sub state machine container 
            smach.StateMachine.add(
                "Search Hint",
                SearchHint(),
                transitions={
                    "no hint": "hint not found", "found hint": "Check Hint Consistency"
                },
                # remapping={"": ""},
            )

            # Add states to the sub state machine container 
            smach.StateMachine.add(
                "Check Hint Consistency",
                CheckHintConsistency(),
                transitions={
                    "inconsistent": "bad hint",
                    "consistent" : "good hint"
                },
                # remapping={"": ""},
            )

        # Add sub state machine to the base state machine container 
        smach.StateMachine.add(
            "Get Hint", GetHint, transitions={"hint not found": "GoTo Room", "bad hint":"GoTo Room", "good hint":"GoTo Oracle"}
        )

        # Add states to the base container
        smach.StateMachine.add(
                "GoTo Oracle",
                GoToOracle(),
                transitions={
                    "reached oracle": "Announce Hypothesis",
                },
                # remapping={"": ""},
            )

        # Add states to the base container
        smach.StateMachine.add(
                "Announce Hypothesis",
                AnnounceHypothesis(),
                transitions={
                    "hypothesis announced": "Oracle (Hypothesis Check)",
                },
                # remapping={"": ""},
            )

        # Add states to the base container
        smach.StateMachine.add(
                "Oracle (Hypothesis Check)",
                OracleCheck(),
                transitions={
                    "hypothesis correct": "Game Won!!!",
                    "hypothesis wrong": "GoTo Room"
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
