#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16
import time

def servo_sweep():
    # Initialize the ROS node
    rospy.init_node('servo_sweep', anonymous=True)
    
    # Create a publisher on the 'servo' topic, with message type 'UInt16'
    pub = rospy.Publisher('servo/angle', UInt16, queue_size=10)

    # Set the rate of publishing (e.g., 10 Hz)
    rate = rospy.Rate(10) # 10 Hz as lidar is publisheding at 10 Hz so 1 data per degree
    
    # Sweep the servo from 0 to 180 and back
    while not rospy.is_shutdown():
        # Sweep from 0 to 180 degrees
        for angle in range(0, 181, 1):
            msg = UInt16()
            msg.data = angle  # Set the servo angle
            pub.publish(msg)  # Publish the message
            rospy.loginfo(f"Publishing: {angle}")  # Optional log
            rate.sleep()  # Sleep to maintain the rate
        
        # Sweep back from 180 to 0 degrees
        for angle in range(180, -1, -1):
            msg = UInt16()
            msg.data = angle  # Set the servo angle
            pub.publish(msg)  # Publish the message
            rospy.loginfo(f"Publishing: {angle}")  # Optional log
            rate.sleep()  # Sleep to maintain the rate

if __name__ == '__main__':
    try:
        servo_sweep()
    except rospy.ROSInterruptException:
        pass
