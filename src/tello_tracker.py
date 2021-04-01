#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceRequest

rospy.init_node('tello_tracker_node')

pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
pub_emergency = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
pub_flattrim = rospy.Publisher('/tello/flattrim', Empty, queue_size=1)
pub_flip = rospy.Publisher('/tello/flip', UInt8, queue_size=1)
pub_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

empty_msg = Empty()
flip_msg = UInt8()
vel_msg = Twist()

rospy.wait_for_service('/track_to_vel_service')
vel_service = rospy.ServiceProxy('/track_to_vel_service', CustomService)
call = CustomServiceRequest()

work_rate = 10
seconds_to_land = 10
seconds_to_takeoff = 5
action_done = False
count = 0
orbit = False
bounce = False
start = rospy.get_time()
print("Tello Tracker Initiated")
rate = rospy.Rate(work_rate)
while not rospy.is_shutdown():    
    result = vel_service(call)
    if not orbit and not bounce:
        vel_msg = result.twist
    if result.tracking:
        start = rospy.get_time()
        if result.id == 0:
            pass
        if result.id == 1 and not action_done:
            flip_msg.data = 0
            for i in range(3):
                pub_flip.publish(flip_msg)
                rospy.sleep(1)
            print("Flipping")
        if result.id == 2 and not action_done:
            for i in range(3):
                pub_takeoff.publish(empty_msg)
                rospy.sleep(1)
            print("Taking Off")
        if result.id == 3:
            print("Tracking")
            pass
        if result.id == 4:
            for i in range(3):
                pub_land.publish(empty_msg)
                rospy.sleep(1)
            print("Landing")
        if result.id == 5:
            for i in range(10):
                pub_emergency.publish(empty_msg)
                rospy.sleep(1)
            print("Emergency")
        if result.id == 6:
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = -0.8
            orbit = True
            print("Orbiting")
        if result.id == 7:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = -1
            vel_msg.angular.z = 0
            orbit = False
            bounce = False
            print("Stopping")
        if result.id == 8:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = -4
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            bounce = True
            print("Stopping")
    else:
        elapsed = rospy.get_time() - start
        if elapsed > 2 and not orbit and not bounce:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
        elif orbit and elapsed > 10:
            orbit = False
        elif bounce and elapsed > 5 and elapsed < 6:
            vel_msg.linear.z = 2
        elif bounce and elapsed > 6:
            bounce = False
            vel_msg.linear.z = 0


    pub_vel.publish(vel_msg)
    rate.sleep()