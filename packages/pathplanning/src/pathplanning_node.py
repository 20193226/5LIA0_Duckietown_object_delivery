#!/usr/bin/env python3

import os
import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float32MultiArray
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

from pathplanning_functions import State, StateMachine, \
    scanning, detected_any, identified, captured, delivering, delivered

class PathPlanningNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PathPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")
        self.duckiedata = np.zeros(1)
        self.initialised = 0
        self.statemachine = StateMachine()
        self.obj_sequence = [0, 0, 0]   #[5, 1, 2]  # hardcoded sequence of objects ids to retrieve (duckie, lemon, orange)
        self.current_obj_cnt = 0                    # object id to retrieve is self.obj_sequence[self.current_obj_cnt]
        self.idx_curr_obj = None                    # tracking index of the object that is currently being tracked, used for indexing self.duckiedata[]
        self.prev_e = 0                             # previous tracking error (for PID control)
        self.prev_int = 0                           # previous integral error term (for PID control)
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
        pub_rate = 3               # publish at 3 Hz
        delta_t = 1/pub_rate
        rate = rospy.Rate(pub_rate)
        while not self.initialised:     # leave this out? Like this, we only start scanning after finding an object...
            pass

        while not rospy.is_shutdown():
            car_control_msg = Twist2DStamped()

            # State machine
            new_state = self.statemachine.state

            if self.statemachine.state == State.SCANNING:
                
                car_control_msg, new_state = scanning(car_control_msg, new_state, self.duckiedata)

            elif self.statemachine.state == State.DETECTED_ANY:

                car_control_msg, new_state = detected_any(car_control_msg, new_state, self.duckiedata,
                                                                             self.obj_sequence[self.current_obj_cnt])

            elif self.statemachine.state == State.IDENTIFIED:
                
                car_control_msg, new_state, e, e_int = identified(car_control_msg, new_state, self.duckiedata,
                                                        self.obj_sequence[self.current_obj_cnt],
                                                        self.prev_e, self.prev_int, delta_t)
                self.prev_e = e
                self.prev_int = e_int

            elif self.statemachine.state == State.CAPTURED:
                
                car_control_msg, new_state = captured(car_control_msg, new_state)

            elif self.statemachine.state == State.DELIVERING:
                pass

            elif self.statemachine.state == State.DELIVERED:

                if self.current_obj_cnt < len(self.obj_sequence):
                    self.current_obj_cnt += 1    # increment obj id to retrieve the next object
                self.prev_e = 0              # reset PID errors
                self.prev_int = 0            # reset PID errors

            # State machine end

            # Publish the car command
            self.pub_car_cmd.publish(car_control_msg)
            rate.sleep()

            # Transition to next state if applicable
            if new_state != self.statemachine.state:
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
