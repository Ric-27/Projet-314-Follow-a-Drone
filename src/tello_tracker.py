#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceRequest

rospy.init_node('tello_tracker_node')
#print("Tello Tracker Projet")

pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
pub_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

empty_msg = Empty()
vel_msg = Twist()

rospy.wait_for_service('/track_to_vel_service')
vel_service = rospy.ServiceProxy('/track_to_vel_service', CustomService)
call = CustomServiceRequest()

rate = rospy.Rate(1)
i = 0
while i < 3:
    pub_takeoff.publish(empty_msg)
    rate.sleep()
    i+=1

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    result = vel_service(call)
    #print(result)
    vel_msg = result.twist
    pub_vel.publish(vel_msg)
    rate.sleep()