#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from projet.srv import CustomService, CustomServiceRequest
import math

rospy.init_node('tello_tracker_node')

pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
pub_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

empty_msg = Empty()
vel_msg = Twist( )# velocity command
pos_msg = Twist() # tracker position
my_vel = Twist() # my velocity

rospy.wait_for_service('/subscription_service')
service = rospy.ServiceProxy('/subscription_service', CustomService)
call = CustomServiceRequest()

sample_time = 0.3 # time in seconds between two samples of positions that we save
work_rate = 10
seconds_to_land = 10
seconds_to_takeoff = 10
in_air = False
count = 0

internal_time1 = rospy.get_rostime()

internal_time_sample_rate = rospy.get_rostime()

seconds_to_initialize = 4
positions = []
date_velocity_queue = []

first = True
#count_init = 0
#init_count = False
#working = False

print("Tello Tracker Initiated")
rate = rospy.Rate(work_rate)

while not rospy.is_shutdown():
    result = service(call)
    #print(internal_time1)
    #print(rospy.get_rostime().secs)
    if not in_air and result.tracking and (rospy.get_rostime().secs - internal_time1.secs) >= seconds_to_takeoff:
        print("Taking Off")
        for i in range(5):
            pub_takeoff.publish(empty_msg)
            rospy.sleep(1)
        internal_time1 = rospy.get_rostime()
        in_air = True
        rospy.sleep(1)
        print("In air")
    if in_air and not result.tracking and (rospy.get_rostime().secs - internal_time1.secs) >= seconds_to_land:
        print("Landing")
        for i in range(5):
            pub_land.publish(empty_msg)
            rospy.sleep(1)
        internal_time1 = rospy.get_rostime()
        in_air = False

    #if (in_air and not result.tracking) or (not in_air and result.tracking):
    #    count += 1
    #if in_air and result.tracking:
        #internal_time1 = rospy.get_rostime()
    
    #if count%work_rate == 0 and count != 0:
    #        print("time elapsed counting: {}".format(count/work_rate))

    pos_msg = result.tracker

    
    if in_air and result.tracking:
        internal_time1 = rospy.get_rostime()

        #init 
        if first:
            positions.append(pos_msg)
            # en supposant en m/s
            vel_msg.linear.x = pos_msg.linear.x/seconds_to_initialize
            vel_msg.linear.y = pos_msg.linear.y/seconds_to_initialize
            vel_msg.linear.z = pos_msg.linear.z/seconds_to_initialize
            first = False
            init_count = True   
            ## add to the date_velocity_queue
            date_velocity_queue.append((vel_msg,rospy.get_rostime().secs)) #stocker vel_msg, date 
            internal_time_sample_rate = rospy.get_rostime()

        
        if (rospy.get_rostime().secs - internal_time_sample_rate.secs) > sample_time:

            ## marqueur en m

            positions.append(pos_msg) #save position sample

            my_vel = result.odom # get it from the topic /tello/odom, but for now assume we did /tello/odom/
            delta_t = (rospy.get_rostime().secs - internal_time_sample_rate.secs)
            vel_msg.linear.x = my_vel.linear.x + (positions[1].linear.x - positions[0].linear.x)/delta_t
            vel_msg.linear.y = my_vel.linear.y + (positions[1].linear.y - positions[0].linear.y)/delta_t
            vel_msg.linear.z = my_vel.linear.z + (positions[1].linear.z - positions[0].linear.z)/delta_t

            #calculate estimated time to target
            #we suppose that
            distance_to_me = math.sqrt((positions[1].linear.x)**2 + (positions[1].linear.y)**2 + (positions[1].linear.z)**2)
            my_vel_norm = math.sqrt((my_vel.linear.x)**2 + (my_vel.linear.y)**2 + (my_vel.linear.z)**2)
            approx_time_to_target = distance_to_me / my_vel_norm

            date_velocity_queue.append((vel_msg,rospy.get_rostime().secs + approx_time_to_target))
            #print(date_velocity_queue)
            #print(date_velocity_queue[0])
            print(date_velocity_queue[0][1])
            internal_time_sample_rate = rospy.get_rostime()
            positions.pop(0)

        if rospy.get_rostime().secs > date_velocity_queue[0][1]: 
            #therefore, we should send the velocity command, because we are approximately in the required position
            pub_vel.publish(date_velocity_queue.pop()[0])



            

    #if init_count:
    #    count_init += 1
    
    #if count_init >= work_rate*seconds_to_initialize:
        #init_count = False
        #count_init = 0
        #working = True

    #if working and result.tracking:
    #    vel_msg.linear.x = (positions[1].linear.x - positions[0].linear.x)*work_rate
    #    vel_msg.linear.y = (positions[1].linear.y - positions[0].linear.y)*work_rate
    #    vel_msg.linear.z = (positions[1].linear.z - positions[0].linear.z)*work_rate
    #    positions.pop(0)

    rate.sleep()