#!/usr/bin/env python3
from typing import Tuple
import math
import numpy as np
import os, rospy, yaml, dataclasses, time
from typing import Optional
from dt_robot_utils import get_robot_name
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from std_msgs.msg import Header

class MappingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MappingNode, self).__init__(node_name=node_name, node_type=NodeType.MAPPING)
        # static parameters

        self.vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self._left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"

	    # form the message
        # temporary data storage
        self._ticks_left_counter = 0
        self._ticks_right_counter =0
        self.ticks_left_send =0
        self.ticks_right_send=0
        self._ticks_left = 0
        self._ticks_right = 0
        self.tick_threshold = 15
        self.time = 0
        #initialise variables
        self.x = 0
        self.y = 0
        self.Objects = 0
        self.last_encoder_message_left = 0
        self.last_encoder_message_right = 0
        self.calledby = 0
        self._turn_angle = 0
        self.distance_traveled = 0.0    
        self.prev_distance = 0
        self.distance_left = 0
        self.distance_right = 0
        self.rate = 1/5 #5 messages every second

        self._odometry_publisher = rospy.Publisher("odometry", Point, queue_size=1)
        #subscriber
        self.wheelencoder_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left, queue_size=1)
        self.wheelencoder_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right, queue_size=1)

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        deltatime = rospy.get_time() - self.time
        
        if self._ticks_left_counter == 0:
            self._ticks_left_counter = data.data
            self.time = rospy.get_time()
        
        else:
            self._ticks_left = data.data - self._ticks_left_counter
            self.ticks_left_send = self.ticks_left_send + self._ticks_left
            self._ticks_left_counter = data.data
            if deltatime >= self.rate:
                message = Point(self.ticks_left_send, self.ticks_right_send, 0)
                self._odometry_publisher.publish(message)
                print("Message has been send by left odometry")
                self.ticks_left_send =0
                self.ticks_right_send=0
                self.time = rospy.get_time()


    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        
        #print("Right encoder ticks = %f" %(data.data))
        deltatime = rospy.get_time() - self.time
        if self._ticks_right_counter == 0:
            self._ticks_right_counter = data.data
            self.time = rospy.get_time()
        else:
        
            self._ticks_right = data.data - self._ticks_right_counter
            self.ticks_right_send = self.ticks_right_send + self._ticks_right
            self._ticks_right_counter = data.data
            if deltatime >= self.rate:
                message = Point(self.ticks_left_send, self.ticks_right_send, 0)
                self._odometry_publisher.publish(message)
                print("Message has been send by right odometry")
                self.ticks_left_send =0
                self.ticks_right_send=0
                self.time = rospy.get_time()

if __name__ == '__main__':
    # create the node
    node = MappingNode(node_name='odometry_node')
    #node.on_shutdown()
    # run node, start movement

    # keep the process from terminating
    rospy.spin()
