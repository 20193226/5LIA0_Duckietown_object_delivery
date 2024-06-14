#!/usr/bin/env python3

import os
import rospy
import numpy as np
from std_msgs.msg import String, Bool, Float32MultiArray
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

from pathplanning_functions import State, StateMachine, \
    scanning, approach, delivered

class PathPlanningNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PathPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")
        self.duckiedata = np.zeros(1)
        self.initialised = 0
        self.statemachine = StateMachine()
        self.obj_sequence = [0, 1]      # hardcoded sequence of objects ids to retrieve (orange, lemon)
        self.current_obj_cnt = 0        # object id to retrieve is self.obj_sequence[self.current_obj_cnt]
        self.dest_obj = 0               # destination is duckie, now: 0, should be 5
        self.idx_curr_obj = None        # tracking index of the object that is currently being tracked, used for indexing self.duckiedata[]
        self.prev_e = 0                 # previous tracking error (for PID control)
        self.prev_int = 0               # previous integral error term (for PID control)
        # self.direction = 0            # keep track in which direction the bot should turn when looking for objects (0: initial, 1: left, 2: right)
        self.no_det_count = 0           # count how many times in a row during approach an object has not been detected
        self.approach_count = 0         # count how many iterations to keep approach for the object to be inside of the claw
        self.run_status = "capture"     # string used in pub_run_status
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

        # Publisher to tell the object detection which model to use
        self.pub_run_stat = rospy.Publisher("run_status", String, queue_size=1)

    def nn_cb(self, nn_output):
        # rospy.loginfo("Observation received")
        self.initialised = 1
        self.duckiedata = nn_output.data
        # for i in range(round(self.duckiedata[0])):
        #     rospy.loginfo("INPUT duck with r,theta, id: %.4f, %.4f, %d",self.duckiedata[i*3+1], self.duckiedata[i*3+2], round(self.duckiedata[i*3+3]))
        

    def pub_car_commands(self):
        pub_rate = 10               # publish at 10 Hz
        delta_t = 1/pub_rate
        rate = rospy.Rate(pub_rate)
        while not self.initialised:
            pass

        while not rospy.is_shutdown():
            car_control_msg = Twist2DStamped()

            # State machine
            new_state = None

            if self.statemachine.state == State.SCANNING:
                
                self.run_status = "capture"
                rospy.loginfo("SCANNING")
                v = 0.6
                car_control_msg, new_state = scanning(car_control_msg, new_state, self.duckiedata, self.obj_sequence[self.current_obj_cnt], v)

            # elif self.statemachine.state == State.DETECTED_ANY:

            #     self.run_status = "capture"
            #     car_control_msg, new_state, direction = detected_any(car_control_msg, new_state, self.duckiedata,
            #                                                                  self.obj_sequence[self.current_obj_cnt], self.direction)
            #     self.direction = direction

            elif self.statemachine.state == State.IDENTIFIED:
                
                rospy.loginfo("IDENTIFIED")
                self.run_status = "capture"
                car_control_msg, new_state, e, e_int, count = approach(car_control_msg, new_state, self.duckiedata,
                                                                self.obj_sequence[self.current_obj_cnt],
                                                                self.prev_e, self.prev_int, delta_t, self.no_det_count, v=0.02)
                self.prev_e = e
                self.prev_int = e_int
                self.no_det_count = count
                self.approach_count = 0
                if new_state == State.CAPTURED:
                    self.run_status = "deliver"
                    self.pub_run_stat.publish(self.run_status)

            elif self.statemachine.state == State.CAPTURED:
                self.run_status = "deliver"
                # self.pub_run_stat.publish(self.run_status)
                
                self.prev_e = 0         # reset PID errors
                self.prev_int = 0       # reset PID errors
                self.no_det_count = 0
                
                rospy.loginfo("CAPTURED")
                # just stop for now. should rotate around 180° until destination object is detected
                car_control_msg.v = 0
                car_control_msg.omega = 0

                if self.approach_count <= 5:
                    # If not close enough yet, go straight
                    self.approach_count += 1
                    car_control_msg.v = 0.02
                    car_control_msg.omega = 0
                else:
                    v = 0.9
                    car_control_msg, new_state = scanning(car_control_msg, new_state, self.duckiedata, self.dest_obj, v)

                    if new_state == State.IDENTIFIED:
                        new_state = State.DELIVERING


            elif self.statemachine.state == State.DELIVERING:
                
                self.run_status = "deliver"
                rospy.loginfo("DELIVERING")
                car_control_msg, new_state, e, e_int, count = approach(car_control_msg, new_state, self.duckiedata,
                                                        self.dest_obj, self.prev_e, self.prev_int,
                                                        delta_t, self.no_det_count, v=0.03)
                self.prev_e = e
                self.prev_int = e_int
                self.no_det_count = count
                self.approach_count = 0
                if new_state == State.CAPTURED:
                    new_state = State.DELIVERED
                    self.run_status = "capture"
                    self.pub_run_stat.publish(self.run_status)
                elif new_state == State.SCANNING:
                    new_state = State.CAPTURED
            

            elif self.statemachine.state == State.DELIVERED:

                self.run_status = "capture"
                # self.pub_run_stat.publish(self.run_status)
                rospy.loginfo("Delivered")
                if self.current_obj_cnt < len(self.obj_sequence):
                    self.current_obj_cnt += 1    # increment obj id to retrieve the next object
                self.prev_e = 0              # reset PID errors
                self.prev_int = 0            # reset PID errors
                self.no_det_count = 0
                # Successfully delivered, stop for now. Should drive back a bit, rotate 180°, then look for the second object to grab
                car_control_msg, new_state = delivered(car_control_msg, new_state)


            # State machine end

            # Publish the car command
            self.pub_car_cmd.publish(car_control_msg)

            # Transition to next state if applicable
            if new_state != None:
                self.statemachine.transition(new_state)
                rospy.loginfo("switching state")

            rate.sleep()

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
