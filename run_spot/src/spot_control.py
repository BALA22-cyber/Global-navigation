#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist  # This message type might vary based on the spot_ros implementation
import time

def spot_walk_turn_stop():
    # Initialize a new ROS node
    rospy.init_node('spot_walk_turn_stop', anonymous=True)

    pub = rospy.Publisher('/spot/command', Twist, queue_size=10)

    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10 Hz

    walk_msg = Twist()
    walk_msg.linear.x = 0.5 
    turn_msg = Twist()
    turn_msg.angular.z = 0.5  
    stop_msg = Twist()  
    rospy.sleep(1)

    #Walk
    walk_duration = 1  
    start_time = time.time()
    while time.time() - start_time < walk_duration:
        pub.publish(walk_msg)
        rate.sleep()
    
    #Turn Left
    turn_duration = 1  
    start_time = time.time()
    while time.time() - start_time < turn_duration:
        pub.publish(turn_msg)
        rate.sleep()

    # Stop
    pub.publish(stop_msg)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    try:
        spot_walk_turn_stop()
    except rospy.ROSInterruptException:
        pass
