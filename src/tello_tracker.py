#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceRequest
import time

rospy.init_node('tello_tracker_node')

pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
pub_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

empty_msg = Empty()
vel_msg = Twist()
pos_msg = Twist()

rospy.wait_for_service('/track_to_vel_service')
vel_service = rospy.ServiceProxy('/track_to_vel_service', CustomService)
call = CustomServiceRequest()

work_rate = 10
seconds_to_land = 10
seconds_to_takeoff = 5
in_air = False
count = 0

seconds_to_initialize = 3
positions = []
first = True
count_init = 0
init_count = False
working = False

print("Tello Tracker Initiated")
rate = rospy.Rate(work_rate)

while not rospy.is_shutdown():
    result = vel_service(call)   
    if not in_air and result.tracking and count >= work_rate*seconds_to_takeoff:
        print("Taking Off")
        for i in range(5):
            pub_takeoff.publish(empty_msg)
            time.sleep(1)
        count = 0
        in_air = True
        time.sleep(3)
    if in_air and not result.tracking and count >= work_rate*seconds_to_land:
        print("Landing")
        for i in range(5):
            pub_land.publish(empty_msg)
            time.sleep(1)
        count = 0
        in_air = False
    if (in_air and not result.tracking) or (not in_air and result.tracking):
        count += 1
    elif in_air and result.tracking:
        count = 0
    
    if count%work_rate == 0 and count != 0:
            print("time elapsed counting: {}".format(count/work_rate))

    pos_msg = result.twist
    
    if in_air and result.tracking:
        positions.append(pos_msg)
    
    if in_air and first and result.tracking:
        vel_msg.linear.x = pos_msg.linear.x/seconds_to_initialize
        vel_msg.linear.y = pos_msg.linear.y/seconds_to_initialize
        vel_msg.linear.z = pos_msg.linear.z/seconds_to_initialize
        first = False
        init_count = True    

    if init_count:
        count_init += 1
    
    if count_init >= work_rate*seconds_to_initialize:
        init_count = False
        count_init = 0
        working = True

    if working and result.tracking:
        vel_msg.linear.x = (positions[1].linear.x - positions[0].linear.x)*work_rate
        vel_msg.linear.y = (positions[1].linear.y - positions[0].linear.y)*work_rate
        vel_msg.linear.z = (positions[1].linear.z - positions[0].linear.z)*work_rate
        positions.pop(0)

    pub_vel.publish(vel_msg)
    rate.sleep()