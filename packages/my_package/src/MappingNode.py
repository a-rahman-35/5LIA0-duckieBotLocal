#!/usr/bin/env python3
from typing import Tuple
import numpy as np
import os, rospy, yaml, dataclasses, time
from typing import Optional
from dt_vl53l0x import VL53L0X #ToF sensor
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Header

class MappingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MappingNode, self).__init__(node_name=node_name, node_type=NodeType.MAPPING)
        # static parameters
        self.wheel_radius = 0.065
        self.duckiebot_width = 0.10
        self.speed_factor = 0.65    # full speed is one, that is 0.65m/s !!v = m/s not needed, maybe for angular?
        self.turn_factor = 0.5
        self.linear_velocity_factor = 1 #self.linear_velocit#self.speed_factor / 2.0
        self.angular_velocity_factor = self.turn_factor * self.speed_factor / self.duckiebot_width
        self.vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"
	    # form the message
        self._v = 0 #m/s
        self.omega = 0 #rad/s
        #initialise variables
        self.prev_v = 0
        self.prev_omega = 0
        self.x = 0
        self.y = 0
        self.theta = 90 #starts off driving towards y direction
        self.prev_x =0
        self.prev_y = 0
        self.prev_theta = 0
        self.last_call_back_time = rospy.get_time()  #size limitations?
        self.last_message_time = -1.0
        self.last_header = None
        self.counter = 0           #used for debug
        self.message_time = 0

        # construct chassis control publisher
        self._chassis_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1) #message needs to be timestamped
        #self._odometry_publisher = rospy.Publisher(self.odometry_topic, TransformStamped, queue_size=1)
        #subscriber
        self._chassis_subscriber = rospy.Subscriber(twist_topic,Twist2DStamped, self.chassis_callback)


        #ToF sensor
        '''
        self._i2c_connectors = rospy.get_param("~connectors", {})
        self._sensor_name = rospy.get_param("~sensor_name")
        self._frequency = int(max(1, rospy.get_param("~frequency", 10)))
        self._mode = rospy.get_param("~mode", "BETTER")
        #self._accuracy = ToFAccuracy.from_string(self._mode)

        self._sensor: Optional[VL53L0X] = self._find_sensor()
        if not self._sensor:
            conns: str = yaml.safe_dump(self._i2c_connectors, indent=2, sort_keys=True)
            self.logerr(f"No VL53L0X device found. These connectors were tested:\n{conns}\n")
            exit(1)
        # create publisher
        self._pub = rospy.Publisher(
            "~range",
            Range,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The distance to the closest object detected by the sensor",
        )
        max_frequency = min(self._frequency, int(1.0 / self._accuracy.timing_budget))
        if self._frequency > max_frequency:
            self.logwarn(
                f"Frequency of {self._frequency}Hz not supported. The selected mode "
                f"{self._mode} has a timing budget of {self._accuracy.timing_budget}s, "
                f"which yields a maximum frequency of {max_frequency}Hz."
            )
            self._frequency = max_frequency
        self.loginfo(f"Frequency set to {self._frequency}Hz.")
        # create timers
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / max_frequency), self._timer_cb) #clashing with other publishers?

    def _find_sensor(self) -> Optional[VL53L0X]:
        for connector in self._i2c_connectors:
            conn: str = "[bus:{bus}](0x{address:02X})".format(**connector)
            self.loginfo(f"Trying to open device on connector {conn}")
            sensor = VL53L0X(i2c_bus=connector["bus"], i2c_address=connector["address"])
            try:
                sensor.open()
            except FileNotFoundError:
                # i2c BUS not found
                self.logwarn(f"No devices found on connector {conn}, the bus does NOT exist")
                continue
            sensor.start_ranging(self._accuracy.mode)
            time.sleep(1)
            if sensor.get_distance() < 0:
                self.logwarn(f"No devices found on connector {conn}, but the bus exists")
                continue
            self.loginfo(f"Device found on connector {conn}")
            return sensor
    def _timer_cb(self, _):
        # detect range
        distance_mm = self._sensor.get_distance()
        # pack observation into a message
        msg = Range(
            header=Header(stamp=rospy.Time.now(), frame_id=f"{self._veh}/tof/{self._sensor_name}"),
            radiation_type=Range.INFRARED,
            field_of_view=self._accuracy.fov,
            min_range=self._accuracy.min_range,
            max_range=self._accuracy.max_range,
            range=distance_mm / 1000,
        )
        # publish
        self._pub.publish(msg)
        # publish display rendering (if it is a good time to do so)
        if self._fragment_reminder.is_time():
            self._renderer.update(distance_mm)
            msg = self._renderer.as_msg()
            self._display_pub.publish(msg)
        '''
    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            message = Twist2DStamped(v = self._v, omega=self.omega)
            self.message_time = rospy.get_time()
            self._chassis_publisher.publish(message)
            rate.sleep()

    def chassis_callback(self, chassis_msg):
        self.last_call_back_time = rospy.get_time()
        #current_time = float(chassis_msg.header.stamp.secs) + 10**(-9) * float(chassis_msg.header.stamp.nsecs)      
        current_time = self.message_time   
        print("Current time = %f" %(current_time))
        if self.last_message_time != -1.0:
            time_diff = current_time - self.last_message_time
            self.compute_odometry(time_diff)

        self.last_header = chassis_msg.header

        self.last_linear_velocity = self.linear_velocity_factor * (chassis_msg.v)
        print("Call back function sees speed %f" %(chassis_msg.v))
        self.last_angular_velocity = self.angular_velocity_factor * (chassis_msg.omega)

        self.last_message_time = current_time

    
    def compute_odometry(self, time_diff):
        #self.chassis_msg.header = self.last_header

        x = 0.0
        y = 0.0
        theta = 0.0

        if (self.last_angular_velocity == 0.0):
            x = time_diff * self.last_linear_velocity
            print("Time diff = %f x = %f" %(time_diff, x))
            y = 0.0
            theta = 0.0
        else:
            radius = self.last_linear_velocity / self.last_angular_velocity
            theta = time_diff * self.last_angular_velocity
            x = radius * np.sin(theta)
            y = radius * (1 - np.cos(theta))
        #Coordinates in duckiebot pose not world pose
        #In order to get to world pose we need the last orientation theta and last pos.
        #Think of best we to construct message for this
        #Need x y and theta. can do calculations outside of message.
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta

        self.theta = self.prev_theta + theta
        self.x = x*np.cos(self.prev_theta) - y*np.sin(self.prev_theta) + self.prev_x        #rotate + shift to correct world pose
        self.y = x*np.sin(self.prev_theta) + y*np.cos(self.prev_theta) + self.prev_y
        print("Duckie is at postion x = %f y = %f theta = %f\n" %(self.x, self.y, self.theta))
        self.counter = self.counter + 1

        if (self.counter == 4):
            self.move(0.5, 0)
        if (self.counter == 5):
            self.move(0, 0)
        if (self.counter == 8):
            self.move(0, 2*np.pi)
        if (self.counter == 10):
            self.move(0, 0)

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._chassis_publisher.publish(stop)

    def move(self, v, omega):
        self._v = self.speed_factor * v
        self.omega = omega
        print("Speed has been set to  v = %f" %(v))
    
if __name__ == '__main__':
    # create the node
    node = MappingNode(node_name='mapping_node')
    # run node, start movement
    node.run()

    # keep the process from terminating
    rospy.spin()
