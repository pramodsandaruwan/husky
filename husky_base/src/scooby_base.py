#!/usr/bin/python

"""
Class for low level control of our car. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
#from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
import time
import Jetson.GPIO as GPIO
import math

class ServoConvert():
    def __init__(self, id=1, center_value=0, range=8000, direction=1):
        self.value      = 0.0
        self.value_out  = center_value
        self._center    = center_value
        self._range     = range
        self._half_range= 0.5*range
        self._dir       = direction
        self.id         = id

        #--- Convert its range in [-1, 1]
        self._sf        = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value      = value_in
        self.value_out  = int(self._dir*value_in*self._half_range + self._center)
        print self.id, self.value_out
        return(self.value_out)

class LowLevelCtrl():
    def __init__(self, wheel_distance=0.6, wheel_diameter=0.4):
        rospy.loginfo("Setting Up the Node...")
        
        rospy.init_node('scooby_llc')

        GPIO.setmode(GPIO.BCM)

        self.PIN_LIGHT = 5
        self.PIN_HORN = 6
        self.gain = 0.75

        GPIO.setup(self.PIN_LIGHT, GPIO.OUT)
        GPIO.setup(self.PIN_HORN, GPIO.OUT)
        
        GPIO.output(self.PIN_LIGHT, GPIO.HIGH)
        GPIO.output(self.PIN_HORN, GPIO.HIGH)

        self._wheel_distance = wheel_distance
        self._wheel_radius = wheel_diameter / 2.0

        self.actuators = {}
        self.actuators['left_a']   = ServoConvert(id=1)
        self.actuators['left_b']   = ServoConvert(id=2, direction=1) #-- positive left
        self.actuators['right_a']  = ServoConvert(id=3)
        self.actuators['right_b']  = ServoConvert(id=4, direction=1)
        rospy.loginfo("> Actuators corrrectly initialized")

        self._servo_msg       = ServoArray()
        #self._imu_data_msg    = Imu()
        self._joint_stat      = JointState()
        self._velocity_msg    = Float32MultiArray()
        for i in range(4): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher 
        self.ros_pub_velocity_array = rospy.Publisher("/wheel_velocity", Float32MultiArray, queue_size=1)
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        #self.ros_pub_imu_data       = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
        self.ros_pub_joint_stat     = rospy.Publisher("/joint_stat", JointState, queue_size=10)


        rospy.loginfo("> Publisher corrrectly initialized")

        #--- Create imu subscriber
        #self.ros_sub_imu            = rospy.Subscriber("/rtimulib_node/imu", Imu, self.send_imu_data_msg)

        #--- Create the Subscriber to Joy commands
        self.ros_sub_joy            = rospy.Subscriber("/joy_teleop/joy", Joy, self.set_actuators_from_joy)

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist          = rospy.Subscriber("/joy_teleop/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        
        rospy.loginfo("> Subscriber corrrectly initialized")

        #--- Get the last time e got a commands
        self._last_time_cmd_rcv     = time.time()
        self._timeout_s             = 5
        self.anglez                 = 0.00

        rospy.loginfo("Initialization complete")

    #def set_angle_from_imu(self,imu_massege):

        #self.imu_data_msg(imu_massege)
        #rospy.loginfo(imu_massege.orientation_covariance[2])

    def set_actuators_from_joy(self,joy_massege):

        #-- Convert joy into actuator values
        if joy_massege.buttons[1] == 1:
            GPIO.output(self.PIN_HORN, GPIO.LOW)
        else:
            GPIO.output(self.PIN_HORN, GPIO.HIGH)

        if joy_massege.buttons[0] == 1:
            GPIO.output(self.PIN_LIGHT, GPIO.HIGH) # - GPIO.input(self.PIN_LIGHT)

        if joy_massege.buttons[2] == 1:
            GPIO.output(self.PIN_LIGHT, GPIO.LOW)


    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.set_cmd_vel(message.linear.x * 0.2,message.angular.z * 0.6)
        rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()

    def calculate_body_turn_radius(self, linear_speed, angular_speed):
        if angular_speed != 0.0:
            body_turn_radius = linear_speed / angular_speed
        else:
            # Not turning, infinite turn radius
            body_turn_radius = None
        return body_turn_radius

    def calculate_wheel_turn_radius(self, body_turn_radius, angular_speed, wheel):

        if body_turn_radius is not None:
            """
            if angular_speed > 0.0:
                angular_speed_sign = 1
            elif angular_speed < 0.0:
                angular_speed_sign = -1
            else:
                angular_speed_sign = 0.0
            """
            if wheel == "right":
                wheel_sign = 1
            elif wheel == "left":
                wheel_sign = -1
            else:
                assert False, "Wheel Name not supported, left or right only."

            wheel_turn_radius = body_turn_radius + ( wheel_sign * (self._wheel_distance / 2.0))
        else:
            wheel_turn_radius = None

        return wheel_turn_radius

    def calculate_wheel_rpm(self, linear_speed, angular_speed, wheel_turn_radius):
        """
        Omega_wheel = Linear_Speed_Wheel / Wheel_Radius
        Linear_Speed_Wheel = Omega_Turn_Body * Radius_Turn_Wheel
        --> If there is NO Omega_Turn_Body, Linear_Speed_Wheel = Linear_Speed_Body
        :param angular_speed:
        :param wheel_turn_radius:
        :return:
        """
        if wheel_turn_radius is not None:
            # The robot is turning
            wheel_rpm = (angular_speed * wheel_turn_radius) / self._wheel_radius
        else:
            # Its not turning therefore the wheel speed is the same as the body
            wheel_rpm = linear_speed / self._wheel_radius

        return wheel_rpm

    def set_cmd_vel(self, linear_speed, angular_speed):

        body_turn_radius = self.calculate_body_turn_radius(linear_speed, angular_speed)

        wheel = "right"
        right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                   angular_speed,
                                                                   wheel)

        wheel = "left"
        left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                  angular_speed,
                                                                  wheel)

        right_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, right_wheel_turn_radius)
        left_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, left_wheel_turn_radius)


        self.left_motors(left_wheel_rpm)
        self.right_motors(right_wheel_rpm)
        self.send_velocity_msg(left_wheel_rpm,right_wheel_rpm)
        #self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)

    def left_motors(self,value):
        value = self.constrain(value,-1.0,1.0)
        if value < 0:
            self.actuators['left_a'].get_value_out(abs(value))
            self.actuators['left_b'].get_value_out(0)

        elif value > 0:
            self.actuators['left_a'].get_value_out(0)
            self.actuators['left_b'].get_value_out(abs(value))
    
        else:
            self.actuators['left_a'].get_value_out(0)
            self.actuators['left_b'].get_value_out(0)

    def right_motors(self,value):
        value = self.constrain(value,-1.0,1.0)
        if value < 0:
            self.actuators['right_a'].get_value_out(abs(value))
            self.actuators['right_b'].get_value_out(0)
            
        elif value > 0:
            self.actuators['right_a'].get_value_out(0)
            self.actuators['right_b'].get_value_out(abs(value))
    
        else:
            self.actuators['right_a'].get_value_out(0)
            self.actuators['right_b'].get_value_out(0)


    def map(self,v, in_min, in_max, out_min, out_max):
        # Check that the value is at least in_min
        if v < in_min:
            v = in_min
        # Check that the value is at most in_max
        if v > in_max:
            v = in_max
        return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def constrain(self,val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['left_a'].get_value_out(0)
        self.actuators['left_b'].get_value_out(0)
        self.actuators['right_a'].get_value_out(0)
        self.actuators['right_b'].get_value_out(0)
        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    #def send_imu_data_msg(self,data):
        #self._imu_data_msg = data
        #self.send_joint_stat_msg()
        #self.ros_pub_imu_data.publish(self._imu_data_msg)

    def send_velocity_msg(self,left,right):
        self._velocity_msg.data = [left, right, self._wheel_distance]
        self.ros_pub_velocity_array.publish(self._velocity_msg)

    def send_joint_stat_msg(self,left,right):
        self._joint_stat.name       = ["front_left_wheel","front_right_wheel","rear_left_wheel","rear_right_wheel"]
        self._joint_stat.position   = [left, right, left, right]
        self._joint_stat.velocity   = [0, 0, 0, 0]
        self._joint_stat.effort     = [0, 0, 0, 0]
        self.ros_pub_joint_stat.publish(self._joint_stat)

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        #print time.time() - self._last_time_cmd_rcv
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #print self._last_time_cmd_rcv, self.is_controller_connected
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    scooby_llc     = LowLevelCtrl()
    scooby_llc.run()
