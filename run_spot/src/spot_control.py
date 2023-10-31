#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
def move_husky():
    rospy.init_node('husky_mover', anonymous=True)
    
    rospy.loginfo("Starting Husky movement...")  # Moved inside the function
    
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # Ensure there are subscribers
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
       rospy.loginfo("Waiting for subscribers...")
       rospy.sleep(1)
    
    move_cmd = Twist()

    # Move forward 1 meter
    move_cmd.linear.x = 0.5 
    move_cmd.angular.z = 0.0
    for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the Husky
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)

    # Turn left by 90 degrees
    move_cmd.angular.z = 1.5708  # π/2 radian/second
    for _ in range(10):  # Assuming 1 rad/s speed for π/2 radian turn and 10Hz publishing rate
        pub.publish(move_cmd)
        rate.sleep()

    # Stop rotation
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)

    # Move forward again by 1 meter
    move_cmd.linear.x = 0.5
    for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the Husky at the end
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    # rospy.loginfo("Moving Husky forward...")

    try:
        move_husky()
    except rospy.ROSInterruptException:
        # rospy.loginfo("Not Moving Husky forward...")
        pass


