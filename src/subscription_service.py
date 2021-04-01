#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from projet.srv import CustomService, CustomServiceResponse
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from constants import LINEAR_THRESHOLD,ANGULAR_THRESHOLD,WEIGHT_LATERAL,WEIGHT_ALTITUDE,WEIGHT_DISTANCE,WEIGHT_ANGULAR_TURN,WEIGHT_ANGULAR_LATERAL,MIN_DISTANCE,MIN_HEIGHT

class Service:
    def __init__(self):
        self.sub_track = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback_track)
        self.sub_odom = rospy.Subscriber('/tello/odom', Odometry, self.callback_odom)
        self.my_service = rospy.Service('/subscription_service', CustomService , self.CB_function)
        self.p_x = 0
        self.p_y = 0
        self.p_z = 0
        self.q_x = 0
        self.q_y = 0
        self.q_z = 0
        self.q_w = 0
        self.tracking = False

        self.odom = Twist()
        print("service server created")

    def CB_function(self,request):
        self.my_response = CustomServiceResponse()
        self.my_response.tracker.linear.x = 0
        self.my_response.tracker.linear.y = 0
        self.my_response.tracker.linear.z = 0
        self.my_response.tracker.angular.x = 0
        self.my_response.tracker.angular.y = 0
        self.my_response.tracker.angular.z = 0
        self.my_response.tracking = False
        self.my_response.odom = self.odom
        
        if self.tracking:
            self.tracking = False
            self.my_response.tracking = True
            
            quaternion = (self.q_x,self.q_y,self.q_z,self.q_w)
            euler = euler_from_quaternion(quaternion)

            l_x = self.p_x * WEIGHT_LATERAL #side
            l_y = -self.p_y* WEIGHT_ALTITUDE #altitude
            l_z = self.p_z * WEIGHT_DISTANCE #distance to marker

            a_z = euler[1] #rotation

            #if abs(l_x) > LINEAR_THRESHOLD:
            #    self.my_response.tracker.linear.x = l_x
            #if abs(l_z) > LINEAR_THRESHOLD:
            #    self.my_response.tracker.linear.y = l_z - MIN_DISTANCE
            #if abs(l_y) > LINEAR_THRESHOLD:
            #    self.my_response.tracker.linear.z = l_y + MIN_HEIGHT
            #if abs(euler[1]) > ANGULAR_THRESHOLD:
            #    self.my_response.tracker.angular.z = a_z * WEIGHT_ANGULAR_TURN
            #    self.my_response.tracker.linear.x += -a_z* WEIGHT_ANGULAR_LATERAL

            self.my_response.tracker.linear.x = self.p_x
            self.my_response.tracker.linear.y = self.p_z
            self.my_response.tracker.linear.z = -self.p_y

        return self.my_response

    def callback_track(self,msg):
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
    
    def callback_odom(self,msg):
        self.odom = msg.twist.twist

rospy.init_node('subscription_service_node')
ss = Service()
rospy.spin()
 