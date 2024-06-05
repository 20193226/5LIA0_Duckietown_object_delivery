#!/usr/bin/env python3

import os
import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float32MultiArray
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

from pathplanning_functions import State, StateMachine, \
    scanning, detected_any, identified

class PathPlanningNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PathPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")
        self.duckiedata = np.zeros(1)
        self.initialised = 0
        self.statemachine = StateMachine()
        self.obj_sequence = [0, 1, 2, 3, 4, 5]      # hardcoded sequence of objects ids to retrieve
        self.current_obj_cnt = 0                    # object id to retrieve is self.obj_sequence[self.current_obj_cnt]
        # construct subscriber
        self.sub_NN_input = rospy.Subscriber('NN_output',
            Float32MultiArray,
            self.nn_cb,
            queue_size=1
        )
        
        # Construct publisher for car control 
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic,
            Twist2DStamped,
            queue_size=1
        )
        

    def nn_cb(self, nn_output):
        # rospy.loginfo("Observation received")
        self.initialised = 1
        self.duckiedata = nn_output.data
        for i in range(round(self.duckiedata[0])):
            rospy.loginfo("INPUT duck with r,theta, id: %.4f, %.4f, %d",self.duckiedata[i*3+1], self.duckiedata[i*3+2], round(self.duckiedata[i*3+3]))
        

    def pub_car_commands(self):
        rate = rospy.Rate(10)  # publish at 10 Hz
        while not self.initialised:     # leave this out? Like this, we only start scanning after finding an object...
            pass

        while not rospy.is_shutdown():
            car_control_msg = Twist2DStamped()

            # State machine
            new_state = None
            if self.statemachine.state == State.SCANNING:
                
                rospy.loginfo("SCANNING")
                car_control_msg = scanning(car_control_msg) # do this before of after if?

                if self.duckiedata[0] > 0:
                    new_state = State.DETECTED_ANY
                    rospy.loginfo("setting new state")

            elif self.statemachine.state == State.DETECTED_ANY:

                rospy.loginfo("DETECTED_ANY")
                car_control_msg = detected_any(car_control_msg)

                for i in range(round(self.duckiedata[0])):
                    # compare ids of detected objects with desired object id:
                    if self.duckiedata[i*3+3] == self.obj_sequence[self.current_obj_cnt]:
                        new_state = State.IDENTIFIED
                        rospy.loginfo("setting new state")

            elif self.statemachine.state == State.IDENTIFIED:
                rospy.loginfo("IDENTIFIED")
                car_control_msg = identified(car_control_msg)
                

            elif self.statemachine.state == State.CAPTURED:
                pass

            elif self.statemachine.state == State.DELIVERING:
                pass

            elif self.statemachine.state == State.DELIVERED:

                self.current_obj_cnt += 1    # increment obj id to retrieve the next object

            # State machine end

            # Publish the car command
            self.pub_car_cmd.publish(car_control_msg)
            rate.sleep()

            # Transition to next state if applicable
            if new_state is not None:
                self.statemachine.transition(new_state)
                rospy.loginfo("switching state")


    def on_shutdown(self):
        stop_msg = Twist2DStamped()
        stop_msg.v = 0
        stop_msg.omega = 0
        self.pub_car_cmd.publish(stop_msg)

if __name__ == '__main__':
    # create the node
    node = PathPlanningNode(node_name='pathplanning')
    
    node.pub_car_commands()
    # keep spinning
    rospy.spin()
