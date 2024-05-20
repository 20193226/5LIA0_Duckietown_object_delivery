#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String, Bool
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

class PathPlanningNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PathPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")
        self.duckiedata = 0
        self.initialised = 0
        # construct subscriber
        self.sub_NN_input = rospy.Subscriber('NN_output',
            Bool,
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
        rospy.loginfo("Observation received '%d'", nn_output.data)
        self.initialised = 1
        self.duckiedata = nn_output.data
        
    def pub_car_commands(self):
        # publish message every 0.1 second (10 Hz)
        rate = rospy.Rate(10)
        self.log("not init")
        while not self.initialised:
            pass
        
        while not rospy.is_shutdown():
            car_control_msg = Twist2DStamped()
            if self.duckiedata == False:
                car_control_msg.v = 0.1
                car_control_msg.omega = 0.0
            else:
                car_control_msg.v = 0.05
                car_control_msg.omega = 1.0

             # Actually publish the message
            self.pub_car_cmd.publish(car_control_msg)
            rate.sleep()
        
    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_car_cmd.publish(stop)

if __name__ == '__main__':
    # create the node
    node = PathPlanningNode(node_name='pathplanning')
    
    node.pub_car_commands()
    # keep spinning
    rospy.spin()
