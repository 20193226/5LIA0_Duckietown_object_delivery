#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

class PathPlanningNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PathPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")
        self.duckiedata = []
        self.initialised = False

        # construct subscriber
        self.sub_NN_input = rospy.Subscriber(
            'NN_output',
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
        self.initialised = True
        self.duckiedata = nn_output.data
        # Log detected objects
        # for i in range(round(self.duckiedata[0])):
        #     rospy.loginfo("INPUT duck with r,theta, id: %.4f, %.4f, %d", self.duckiedata[i*3+1], self.duckiedata[i*3+2], round(self.duckiedata[i*3+3]))
        
    def pub_car_commands(self):
        rate = rospy.Rate(10)  # publish at 10 Hz
        while not rospy.is_shutdown():
            car_control_msg = Twist2DStamped()

            if self.initialised and self.duckiedata[0] > 0:
                # Object detected, stop the car
                car_control_msg.v = 0.0
                car_control_msg.omega = 0.0
            else:
                # No object detected, move forward
                car_control_msg.v = 0.2  # Set a forward velocity
                car_control_msg.omega = 0.0

            # Publish the car command
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
