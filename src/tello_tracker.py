#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceRequest

rospy.init_node('tello_tracker_node')

pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
pub_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

empty_msg = Empty()
vel_msg = Twist()

rospy.wait_for_service('/track_to_vel_service')
vel_service = rospy.ServiceProxy('/track_to_vel_service', CustomService)
call = CustomServiceRequest()

work_rate = 10
seconds_to_land = 10
seconds_to_takeoff = 5
in_air = False
count = 0

print("Tello Tracker Initiated")
while not rospy.is_shutdown():
    rate = rospy.Rate(work_rate)
    result = vel_service(call)   
    if not in_air and result.tracking and count >= work_rate*seconds_to_takeoff:
        print("Taking Off")
        rate = rospy.Rate(1)
        for i in range(5):
            pub_takeoff.publish(empty_msg)
            rate.sleep()
        count = 0
        in_air = True
    if in_air and not result.tracking and count >= work_rate*seconds_to_land:
        print("Landing")
        rate = rospy.Rate(1)
        for i in range(5):
            pub_land.publish(empty_msg)
            rate.sleep()
        count = 0
        in_air = False
    if (in_air and not result.tracking) or (not in_air and result.tracking):
        count += 1
    elif in_air and result.tracking:
        count = 0
    
    if count%work_rate == 0 and count != 0:
            print("time elapsed counting: {}".format(count/work_rate))
    vel_msg = result.twist
    pub_vel.publish(vel_msg)
    rate.sleep()