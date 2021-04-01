#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceResponse
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from constants import LINEAR_THRESHOLD,ANGULAR_THRESHOLD,WEIGHT_LATERAL,WEIGHT_ALTITUDE,WEIGHT_DISTANCE,WEIGHT_ANGULAR_TURN,WEIGHT_ANGULAR_LATERAL,MIN_DISTANCE,MIN_HEIGHT
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
        self.id = 100
        self.tracking = False
        print("service server created")

    def CB_function(self,request):

        self.my_response = CustomServiceResponse()
        self.my_response.twist.linear.x = 0
        self.my_response.twist.linear.y = 0
        self.my_response.twist.linear.z = 0
        self.my_response.twist.angular.x = 0
        self.my_response.twist.angular.y = 0
        self.my_response.twist.angular.z = 0
        self.my_response.tracking = False
        self.my_response.id = 100    
        
        if self.tracking:
            self.tracking = False
            self.my_response.id = self.id
            self.my_response.tracking = True
            
            if self.id == 3:
                quaternion = (self.q_x,self.q_y,self.q_z,self.q_w)
                euler = euler_from_quaternion(quaternion)

                l_x = self.p_x * WEIGHT_LATERAL #side
                l_y = -self.p_y* WEIGHT_ALTITUDE #altitude
                l_z = self.p_z * WEIGHT_DISTANCE #distance to marker

                a_z = euler[1] #rotation

                if abs(l_x) > LINEAR_THRESHOLD:
                    self.my_response.twist.linear.x = l_x
                if abs(l_z) > LINEAR_THRESHOLD:
                    self.my_response.twist.linear.y = l_z - MIN_DISTANCE
                if abs(l_y) > LINEAR_THRESHOLD:
                    self.my_response.twist.linear.z = l_y + MIN_HEIGHT
                if abs(euler[1]) > ANGULAR_THRESHOLD:
                    self.my_response.twist.angular.z = a_z * WEIGHT_ANGULAR_TURN
                    self.my_response.twist.linear.x += -a_z* WEIGHT_ANGULAR_LATERAL

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
            self.id = msg.markers[0].id
        else:
            self.p_x = 0
            self.p_y = 0
            self.p_z = 0
            self.q_x = 0
            self.q_y = 0
            self.q_z = 0
            self.q_w = 0
            self.id = 100

rospy.init_node('track_to_vel_service_node')
ss = Service()
rospy.spin()
 