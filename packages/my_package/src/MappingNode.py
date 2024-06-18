#!/usr/bin/env python3
from typing import Tuple
import math
import numpy as np
import os, rospy, yaml, dataclasses, time
from typing import Optional
from dt_robot_utils import get_robot_name
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Header
SENSOR_NAME: str = "front_center"

class Object:
    def __init__(self, type, x, y):
        self.type = type
        self.x = x
        self.y = y
        self.status = "Not Picked up"

class MappingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MappingNode, self).__init__(node_name=node_name, node_type=NodeType.MAPPING)
        # static parameters
        self.ToF_distance=  0.4 #in meters

        self.vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
        self._left_encoder_topic = f"/{self.vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self.vehicle_name}/right_wheel_encoder_node/tick"
	    # form the message
        self._v = 0 #m/s
        self.omega = 0 #rad/s
        # temporary data storage
        self._ticks_left_counter = 0
        self._ticks_right_counter =0
        self._ticks_left = 0
        self._ticks_right = 0
  
        self.time = 0
        #initialise variables
        self.x = 0
        self.y = 0
        self.Objects = 0
        self.points = []

        self.i = 0
        self.Kp = 0.3  # Proportional gain
        self.Ki = 0.05  # Integral gain
        self.Kd = 0.1  # Derivative gain
        self.integral_error = 0.0
        self.prev_error = 0.0

        self._turn_angle = 0
        self.distance_traveled = 0.0
        self.actual_distance_traveled =0
        self.distance_left = 0
        self.distance_right = 0
        self.theta = 0 #starts off driving towards y direction
        self.mapnotmade =0
        self.distance_to_drive =0
        self.angle_to_turn =0
        self.turn_direction = "right"
        self.Map_state = "Initializing"
        self.lastrun = False
        self.movement_state = "init"

        # construct chassis control publisher
        self._chassis_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1) #message needs to be timestamped
        #self._odometry_publisher = rospy.Publisher(self.odometry_topic, TransformStamped, queue_size=1)
        #subscriber
        #self._chassis_subscriber = rospy.Subscriber(twist_topic,Twist2DStamped, self.chassis_callback)
        self._tof_subscriber = rospy.Subscriber(f"/{self.vehicle_name}/{SENSOR_NAME}_tof_driver_node/range", Range, self.Brain)
        self.wheelencoder_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.wheelencoder_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def run(self):
        self.Map_state = "Determine target"

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")

        if self._ticks_left_counter == 0:
            self._ticks_left_counter = data.data
        
        else:
            self._ticks_left = data.data - self._ticks_left_counter
            self.odometry_update()
            self._ticks_left = 0
            self._ticks_left_counter = data.data
            
    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")

        if self._ticks_right_counter == 0:
            self._ticks_right_counter = data.data
        else:
            self._ticks_right = data.data - self._ticks_right_counter
            self.odometry_update()
            self._ticks_right =0
            self._ticks_right_counter = data.data
            
    def MakeMap(self):
        Block1 = Object("Purple", 10, 2)
        Block2 = Object("Pink", 1.1, 0)
        Block3 = Object("Green",1,0)
        Block4 = Object("Green",10,1)
        Objects = [Block1, Block2, Block3, Block4]
        return Objects
    
    def Brain(self, ToF_msg):
        if self.Map_state == "Determine target":
            if self.mapnotmade == 0:
                self.Objects = self.MakeMap()
                self.mapnotmade =1
                for i in range(len(self.Objects)):
                    self.points.append([self.Objects[i].x,self.Objects[i].y])
            
            drivetopoint = self.find_path(self.points)
            self.points.pop(self.points.index(drivetopoint))
            print("Closest point is %f, %f" %(drivetopoint[0],drivetopoint[1]))
            self.distance_to_drive = self.find_distance(drivetopoint)
            self.angle_to_turn = self.find_angle(drivetopoint)
            print("Driving distance %f and angle %f" %(self.distance_to_drive,self.angle_to_turn))

            #jurre his pathfinding function
            #ordered list of points
            #directions to get there
            self.move(0.3,np.pi)
            self.Map_state = "turn"

        elif self.Map_state == "drive":
            print("x= %f y= %f" %(self.x, self.y))
            if ToF_msg.range <= self.ToF_distance:
                print("Current position x= %f, y=%f, theta=%f" %(self.x, self.y, self.theta))
                self.Map_state = "avoid_obstacle"
                self.original_theta = self.theta
            elif(self.drive_distance(self.distance_to_drive)==1):
                self.move(0,0)
                self.Map_state = "Done"
            else:
                if self.movement_state != "driving":
                    self.move(0.3, 0)
                    self.movement_state = "driving"
                
                elif (self.i % 2 == 0):
                    error = self.original_theta - self.theta
                    self.integral_error += error
                    derivative_error = error - self.prev_error

                    control_effort = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
                    self.prev_error = error
                    print("debug: error= %f original theta = %f current theta %f \n" %(error,self.original_theta, self.theta))
                    self.move(0.3, control_effort)
                
                self.i + 1

        elif self.Map_state == "turn":
            if ToF_msg.range <= self.ToF_distance:
                dummy =1
                #lets assume turn will be made quick no enough so not needed
            elif(self.turning(self.angle_to_turn)):
                self.move(0.3, 0)
                self.actual_distance_traveled =0
                self.Map_state = "drive"
                #needs to look for object now? dont think       
        elif self.Map_state == "avoid_obstacle":
            #approach object get distance from ToF sensor
            #store location
            self.smooth_turning(ToF_msg.range)
            if ToF_msg.range > self.ToF_distance:
                self.Map_state = "return_to_path"

        elif self.Map_state == "return_to_path":
            self.return_to_path()
            print("original theta = %f current theta %f \n" %(self.original_theta, self.theta))
            if ToF_msg.range < self.ToF_distance:
                self.Map_state = "avoid_obstacle"
            elif abs(self.original_theta - self.theta) < 0.1:  # Adjust the threshold as necessary
                #$self.Map_state = "drive"
                print("stop\n")
                self.move(0, 0)

    def Map_Callback(self, ToF_msg):
        #Need to wait until everything is initialized
        degrees = np.rad2deg(self.theta)
        print("Distance Travelled = %f" %(degrees))
        if self.Map_state == "drive":
            if ToF_msg.range <= self.ToF_distance or self.theta  >= np.pi:
                self.move(0, 0)
                print("Current position x= %f, y=%f, theta=%f" %(self.x, self.y, self.theta))
                if self.lastrun == True:
                    #mapping is done, start path planning
                    #go to first object found
                    self.Map_state = "path_planning"
                else:
                    self.Map_state = "done"

            #Maybe not ideal this keeps calling odometry function?

        elif self.Map_state == "turn_start":
            self._turn_angle = self.theta + 90
            #set initial turning angle. What happens if 350 + 90?
            self.turning(self.theta, self._turn_angle)
            self.Map_state = "turning"

        elif self.Map_state == "turning":
            self.turning(self.theta, self._turn_angle)
            #check if cardboard wall is detected
            if ToF_msg.range <= self.ToF_distance:
                self.lastrun = True

            #if detected, last run is starting  ->continue
            #else continue
            #turn 90 degrees
            #Move straight

        elif self.Map_state == "object_detected":
            #approach object get distance from ToF sensor
            #store location
            x = 0  
            y = 0       
            type = "idkyetbruh"
            NewObject = Object(type, x, y)
            self.Objects.append(NewObject)
        
            #avoid object
            #continue driving
            self.move(0.5, 0)  #Move straight
            self.Map_state="drive"

    def odometry_update(self):
        #print("Ticks Left=%f Ticks Right=%f" %(self._ticks_left, self._ticks_right))
        self.distance_left = 2*np.pi*0.0318*(self._ticks_left)/135
        self.distance_right = 2*np.pi*0.0318*(self._ticks_right)/135
        self.distance_traveled = ((self.distance_left + self.distance_right)/2)
        
        self.actual_distance_traveled = self.actual_distance_traveled + self.distance_traveled

        angle_turned = (2*np.pi*0.0318*(self._ticks_right)/135 - 2*np.pi*0.0318*(self._ticks_left)/135)/0.1
        self.x = self.x + self.distance_traveled*np.cos(self.theta)
        self.y = self.y + self.distance_traveled*np.sin(self.theta)

        if (self.theta + angle_turned) > np.pi*2:
            self.theta = (self.theta+angle_turned)%(np.pi*2)
        else:
            self.theta = self.theta + angle_turned
        printer = np.rad2deg(self.theta)
        print("Theta=%f" %(printer))
        

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._chassis_publisher.publish(stop)

    def move(self, v, omega):
        self._v = v
        self.omega = omega
        message = Twist2DStamped(v = self._v, omega=self.omega)
        self._chassis_publisher.publish(message)
        print("Speed has been set to  v = %f" %(v))
    
    def turning(self, angle_aimed):         #while turning its still moving forward take this into account
        degrees = np.rad2deg(self.theta)
        if degrees >= angle_aimed:
            self.move(0,0)
            print("Degrees=%f" %(degrees))
            return 1
        else:
            return 0   

    def drive_distance(self, distance):
        if self.actual_distance_traveled>= distance:
            self.move(0,0)
            return 1
        else:
            return 0
    def find_distance(self,ending_loc):
        distance = math.sqrt((ending_loc[0]-self.x)**2+(ending_loc[1]-self.y)**2)
        return distance

    def find_angle(self,ending_loc):
        angle = math.atan2((ending_loc[1]-self.x),ending_loc[0]-self.y)
        angle = math.degrees(angle)
        return angle

    def find_path(self, points):
        distances = []
        # starting_point = [self.x,self.y]
        for i in range(len(points)):
            distances.append(self.find_distance(points[i]))
            # print(str(distances))
            # print(distances.index(min(distances)))
        closest = points[distances.index(min(distances))]
        return closest
    
    def smooth_turning(self, distance):
        TURN_FACTOR = 5
        BASE_SPEED = 0.3
        SAFE_DISTANCE = 0.4  # meters
        MIN_SPEED = 0.1
        MAX_SPEED = 0.3
        CLOSEST_DISTANCE = 0.3 

        # Proportional control to determine turning angle
        error = (0.1+SAFE_DISTANCE) - distance
        turn_rate = 0.5 + TURN_FACTOR * error / SAFE_DISTANCE

        if distance < CLOSEST_DISTANCE:
            speed = MIN_SPEED
        elif distance > SAFE_DISTANCE:
            speed = MAX_SPEED
        else:
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (1 - (error / (SAFE_DISTANCE - CLOSEST_DISTANCE)))

        # Ensure the speed is within the desired range
        speed = max(MIN_SPEED, min(MAX_SPEED, speed))
	
	
        # Ensure the turn rate is within a reasonable range
        turn_rate = max(min(turn_rate, np.pi), -np.pi)

        if self.turn_direction == 'left':
            omega = turn_rate
        else:
            omega = -turn_rate
        
        print("Omega = %f \n" %(omega))

        self.move(speed, omega)

    def return_to_path(self):
        current_heading = self.theta  # Assuming theta is updated by odometry_update
        heading_error = self.original_theta - current_heading

        if abs(heading_error) > 0.1:  # Adjust the threshold as necessary
            if heading_error > 0:
                self.move(0.3, np.pi/2)  # Adjust omega for turning left while moving forward
            else:
                self.move(0.3, -np.pi/2)  # Adjust omega for turning right while moving forward
        else:
            self.move(0.0, 0)  # Move forward once aligned



    
if __name__ == '__main__':
    # create the node
    node = MappingNode(node_name='mapping_node')
    node.run()
    # run node, start movement

    # keep the process from terminating
    rospy.spin()
