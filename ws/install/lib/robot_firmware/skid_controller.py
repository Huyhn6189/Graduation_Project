#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray,Float64MultiArray
from geometry_msgs.msg import Quaternion, Twist
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import tf


# lineare maxx : 0.08
# 	min 0.015
# angular z max 0.5
# 	min 0.15
class Node:
    def __init__(self):
        rospy.init_node("robot_firmware")

        rospy.Subscriber("pose_encod",Int32MultiArray,self.pose_encod_cb)
        rospy.Subscriber("delta_enc",Int32MultiArray,self.delta_encod_cb)
        rospy.Subscriber("cmd_vel",Twist,self.cmd_vel_cb)

        self.duty_pub = rospy.Publisher("duty_motor",Int32MultiArray,queue_size=10)
        self.direct_pub = rospy.Publisher("direct_motor",Int32MultiArray,queue_size=10)
        self.vel_wheel_real_pub = rospy.Publisher("vel_wheel_real",Float64MultiArray,queue_size=10)
        self.vel_wheel_target_pub = rospy.Publisher("vel_wheel_target",Float64MultiArray,queue_size=10)
        self.pub_z = rospy.Publisher('/rotate_z', Float64, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.last_enc_time = rospy.Time.now()

        # front_left, front_right , rear_left, rear_right
        
        self.pose_ecoder_data_now = [0 , 0, 0, 0]
        self.pose_ecoder_data_prev = [0 , 0, 0, 0]
        self.delta_ecoder_data = [0.0 , 0.0, 0.0, 0.0]
        self.vel_wheel_target = [0.0 , 0.0, 0.0, 0.0]
        # self.vel_wheel_real = [0.0 , 0.0, 0.0, 0.0]

        self.delta_travel_wheel= [0.0 , 0.0, 0.0, 0.0]
        self.vel_wheel_real= [0.0 , 0.0, 0.0, 0.0]

        # PID Params
        self.kp = [20.8, 19.5, 8.5,7.5]
        self.ki = [7.6, 7.1, 03.5, 01.8]
        self.kd = [0.04 , 0.03, 0.03, 0.02]

        # Error 
        self.error_now = [0.0 , 0.0, 0.0, 0.0]
        self.error_last = [0.0 , 0.0, 0.0, 0.0]
        self.error_last_last = [0.0 , 0.0, 0.0, 0.0]
        self.error_all =[0.0 , 0.0, 0.0, 0.0]

        self.twist_target = Twist()

        # Dimension of robot 
        self.wheelSeparation = 0.38
        self.wheelSeparationLength = 100
        self.diameter_wheel = 0.2
        self.ticks_per_round = 3350
        self.ticks_per_metter = 0.0

        self.pi_ = 3.14159


        # pose of robot 
        self.position_x = 0.0
        self.position_y = 0.0
        self.angle_z = 0.0

        # Vel of robot
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # Duty  

        self.duty_data = [0.0, 0.0, 0.0, 0.0]
        self.direct_data =[0.0 , 0.0, 0.0, 0.0]

        self.duty_data_pub = Int32MultiArray()
        self.direct_data_pub = Int32MultiArray()

        
        # left_ticks_meter = 1.0 / (pi_*diameter_wheel) * float (left_ticks_round);
        # right_ticks_meter = 1.0 / (pi_*diameter_wheel) * float (right_ticks_round);

        self.ticks_per_metter  = 1.0 / (self.pi_ * self.diameter_wheel) * self.ticks_per_round

    def cmd_vel_cb(self,msg):

        self.twist_target = msg
        
    def pose_encod_cb(self,msg):
        # print(msg)
        self.pose_ecoder_data_now = msg.data

    def delta_encod_cb(self,msg):
        self.delta_encod_data  = msg.data
        self.pose_right_now_encod = msg.data[0]
        self.pose_left_now_encod = msg.data[1]

    def run(self):

        r_time = rospy.Rate(10)

        linear_x = 0.0 
        linear_y = 0.0
        angular_z = 0.0

        while not rospy.is_shutdown():
            
            linear_x = self.twist_target.linear.x
            # linear_y = self.twist_target.linear.y 
            angular_z = self.twist_target.angular.z

            print("lineare   x :;; ", linear_x)
            print("angular_z   x :;; ", angular_z)

            vel_left =  ((2 * linear_x) - (angular_z * self.wheelSeparation ))/ 2 
            vel_right = ((2 * linear_x) + (angular_z * self.wheelSeparation ))/ 2 
            
            self.vel_wheel_target[0] = self.ticks_per_metter * vel_right    # front right
            self.vel_wheel_target[1] = self.ticks_per_metter * vel_left     # front left
            self.vel_wheel_target[2] = self.ticks_per_metter * vel_right    # trrrear right
            self.vel_wheel_target[3] = self.ticks_per_metter * vel_left        # rear left
            
            # self.vel_wheel_target[0] = self.ticks_per_metter * (linear_x - linear_y - (self.wheelSeparation + self.wheelSeparationLength)*angular_z)
            # self.vel_wheel_target[1] = self.ticks_per_metter * (linear_x + linear_y + (self.wheelSeparation + self.wheelSeparationLength)*angular_z)
            # self.vel_wheel_target[2] = self.ticks_per_metter * (linear_x + linear_y - (self.wheelSeparation + self.wheelSeparationLength)*angular_z)
            # self.vel_wheel_target[3] = self.ticks_per_metter * (linear_x - linear_y + (self.wheelSeparation + self.wheelSeparationLength)*angular_z)
            
            print(" Vel wheels target : " ,self.vel_wheel_target)

            self.delta_ecoder_data[0]= self.pose_ecoder_data_now[0] - self.pose_ecoder_data_prev[0]
            self.delta_ecoder_data[1]= self.pose_ecoder_data_now[1] - self.pose_ecoder_data_prev[1]
            self.delta_ecoder_data[2]= self.pose_ecoder_data_now[2] - self.pose_ecoder_data_prev[2]
            self.delta_ecoder_data[3]= self.pose_ecoder_data_now[3] - self.pose_ecoder_data_prev[3]
            
            print("Delta encoder Dat : " , self.delta_ecoder_data)

            self.delta_travel_wheel[0] = self.delta_ecoder_data[0] / self.ticks_per_metter
            self.delta_travel_wheel[1] = self.delta_ecoder_data[1] / self.ticks_per_metter
            self.delta_travel_wheel[2] = self.delta_ecoder_data[2] / self.ticks_per_metter
            self.delta_travel_wheel[3] = self.delta_ecoder_data[3] / self.ticks_per_metter
            print("Delta travel wheel : ", self.delta_travel_wheel)

            current_time = rospy.Time.now()
            delta_t = (current_time - self.last_enc_time).to_sec()
            self.last_enc_time = current_time

            self.vel_wheel_real[0] = self.delta_travel_wheel[0] / delta_t
            self.vel_wheel_real[1] = self.delta_travel_wheel[1] / delta_t
            self.vel_wheel_real[2] = self.delta_travel_wheel[2] / delta_t
            self.vel_wheel_real[3] = self.delta_travel_wheel[3] / delta_t
            print("Velocity of   wheel  real : ", self.vel_wheel_real)
            
            self.pose_ecoder_data_prev[0] = self.pose_ecoder_data_now[0]
            self.pose_ecoder_data_prev[1] = self.pose_ecoder_data_now[1]
            self.pose_ecoder_data_prev[2] = self.pose_ecoder_data_now[2]
            self.pose_ecoder_data_prev[3] = self.pose_ecoder_data_now[3]
            # Delta Origin of Robot 
            # travel_x = (self.delta_travel_wheel[0] + self.delta_travel_wheel[1] + self.delta_travel_wheel[2]+ self.delta_travel_wheel[3]) / 4.0
            # travel_y = (- self.delta_travel_wheel[0] + self.delta_travel_wheel[1] + self.delta_travel_wheel[2] - self.delta_travel_wheel[3]) / 4.0
            # travel_theta = (-self.delta_travel_wheel[0] + self.delta_travel_wheel[1] - self.delta_travel_wheel[2]+ self.delta_travel_wheel[3]) / (2 * (self.wheelSeparation + self.wheelSeparationLength))

            travel_left = self.delta_travel_wheel[1]
            travel_right = self.delta_travel_wheel[0]

            travel_x = (travel_right + travel_left ) / 2.0 
            travel_theta =  (travel_right - travel_left ) / (self.wheelSeparation )

            self.angle_z = self.angle_z + travel_theta
            if self.angle_z >= 6.28:
                self.angle_z = self.angle_z - 6.28
            if self.angle_z <= - 6.28:
                self.angle_z = self.angle_z + 6.28

            delta_x = travel_x* math.cos(self.angle_z) 
            delta_y = travel_x* math.sin(self.angle_z) 

            self.position_x = self.position_x + delta_x
            self.position_y = self.position_y + delta_y
            # self.pose.x += deltaXTravel*cos(self.pose.theta) - deltaYTravel*sin(self.pose.theta)
            # self.pose.y += deltaYTravel*cos(self.pose.theta) + deltaXTravel*sin(self.pose.theta)
            # theta = (self.pose.theta + deltaTheta) % (2*pi)
            
            self.calculate_PID(0,vel_right,self.vel_wheel_real[0],delta_t) 
            self.calculate_PID(1,vel_left,self.vel_wheel_real[1],delta_t)
            self.calculate_PID(2,vel_right,self.vel_wheel_real[2],delta_t)
            self.calculate_PID(3,vel_left,self.vel_wheel_real[3],delta_t)
            
            duty_array = Int32MultiArray()
            direct_array = Int32MultiArray()

            vel_real_data = Float64MultiArray()
            vel_target_data = Float64MultiArray()

            vel_real_data.data =  self.vel_wheel_real
            vel_target_data.data = [vel_left,vel_right,vel_left,vel_right]




            direct_array.data = [int(self.duty_data[0]),int(self.duty_data[1]),
                                int(self.duty_data[2]),int(self.duty_data[3])]
            

            duty_array.data =  [int(abs(self.duty_data[0])),int(abs(self.duty_data[1])),
                                int(abs(self.duty_data[2])),int(abs(self.duty_data[3]))]

            # direct_array.data = [-100,-100,-100,-100]
            # duty_array.data = [100,100,100,100]


            print("duty ", self.duty_data)
            print("direct ",direct_array.data )
            self.direct_pub.publish(direct_array)
            self.duty_pub.publish(duty_array)

            self.vel_wheel_real_pub.publish(vel_real_data)
            self.vel_wheel_target_pub.publish(vel_target_data)

            self.odom_pub_CB(0.0,0.0,0.0)
            self.pub_tf_odom(self.position_x,self.position_y, self.angle_z)

            r_time.sleep()

    def calculate_PID(self,stt,vel_target, vel_real,delta_time):

        self.error_last_last[stt]  = self.error_last[stt]
        self.error_last[stt] = self.error_now[stt]
        self.error_now[stt] = (vel_target - vel_real)

        P_ = self.kp[stt] *( self.error_now[stt]);
        I_ = 0.5 *self.ki[stt]  * delta_time *( self.error_now[stt] + self.error_last[stt])
        D_ = self.kd[stt] / delta_time * (self.error_now[stt] - 2*self.error_last[stt] +self.error_last_last[stt]);
        
        self.duty_data[stt] = self.duty_data[stt] + P_ + I_ + D_

        if (self.duty_data[stt] > 200.0) : 
            self.duty_data[stt] = 200

        if (self.duty_data[stt] < -200) :
            self.duty_data[stt] = -200
        
        if vel_target == 0.0:
            self.duty_data[stt] = 0
        


    def odom_pub_CB(self,linear_x,linear_y,rotate_z):

        odom_quat = Quaternion()
        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.angle_z)
        a = Float64()
        a.data=self.angle_z /3.14 * 180.0
        self.pub_z.publish(a)

        rospy.loginfo(self.angle_z)

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = 0.0
        # odom_msg.pose.pose.orientation = odom_quat

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]

        odom_msg.pose.covariance[0]= 0.01
        odom_msg.pose.covariance[7]= 0.01
        odom_msg.pose.covariance[14]= 1e-12
        odom_msg.pose.covariance[21]= 1e-12
        odom_msg.pose.covariance[28]= 1e-12
        odom_msg.pose.covariance[35]= 0.01
        
        odom_msg.twist.twist.linear.x = linear_x
        odom_msg.twist.twist.linear.x = linear_y
        odom_msg.twist.twist.angular.z = rotate_z

        odom_msg.twist.covariance[0]= 0.01
        odom_msg.twist.covariance[7]= 1e-12
        odom_msg.twist.covariance[14]= 1e-12
        odom_msg.twist.covariance[21]= 1e-12
        odom_msg.twist.covariance[28]= 1e-12
        odom_msg.twist.covariance[35]= 0.01

        # print odom_msg
        self.odom_pub.publish(odom_msg)
        
    def pub_tf_odom(self,pose_x,pose_y,rotate_z):
        
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, rotate_z)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (pose_x, pose_y, 0.),
            odom_quat,
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )
if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")











