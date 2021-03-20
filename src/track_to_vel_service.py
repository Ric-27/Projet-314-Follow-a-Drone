#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceResponse
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

MULTIPLIER = 1.5
LINEAR_THRESHOLD = 0.1
ANGULAR_THRESHOLD = 0.8

class Service:
    def __init__(self):
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback)
        self.my_service = rospy.Service('/track_to_vel_service', CustomService , self.CB_function)
        self.p_x = 0
        self.p_y = 0
        self.p_z = 0
        self.q_x = 0
        self.q_y = 0
        self.q_z = 0
        self.q_w = 0
        self.tracking = False
        print("service server created")

    def CB_function(self,request):
        if self.tracking:
            print("TRACKING MARKER")
            self.tracking = False
        self.my_response = CustomServiceResponse()
        self.my_response.twist.linear.x = 0
        self.my_response.twist.linear.y = 0
        self.my_response.twist.linear.z = 0
        self.my_response.twist.angular.x = 0
        self.my_response.twist.angular.y = 0
        self.my_response.twist.angular.z = 0

        if abs(self.p_x) > LINEAR_THRESHOLD:
            self.my_response.twist.linear.x = -self.p_x
        if abs(self.p_z) > LINEAR_THRESHOLD:
            self.my_response.twist.linear.y = self.p_z - 1
        if abs(self.p_y) > LINEAR_THRESHOLD:
            self.my_response.twist.linear.z = -self.p_y
        
        quaternion = (self.q_x,self.q_y,self.q_z,self.q_w)
        euler = euler_from_quaternion(quaternion)
        #self.my_response.twist.angular.x = euler[0] #r
        #self.my_response.twist.angular.y = euler[2] #y
        if abs(euler[1]) > ANGULAR_THRESHOLD:
            self.my_response.twist.angular.z = euler[1] * MULTIPLIER
            self.my_response.twist.linear.x = -0.5

        return self.my_response

    def callback(self,msg):
        if len(msg.markers) > 0:
            self.tracking = True
            self.p_x = msg.markers[0].pose.pose.position.x
            self.p_y = msg.markers[0].pose.pose.position.y
            self.p_z = msg.markers[0].pose.pose.position.z
            self.q_x = msg.markers[0].pose.pose.orientation.x
            self.q_y = msg.markers[0].pose.pose.orientation.y
            self.q_z = msg.markers[0].pose.pose.orientation.z
            self.q_w = msg.markers[0].pose.pose.orientation.w
        else:
            self.p_x = 0
            self.p_y = 0
            self.p_z = 0
            self.q_x = 0
            self.q_y = 0
            self.q_z = 0
            self.q_w = 0

rospy.init_node('track_to_vel_service_node')
ss = Service()
rospy.spin()
 