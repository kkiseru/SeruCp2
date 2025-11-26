#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

front_distance = float('inf')

def scan_callback(scan_data):
    """Update global front distance from LIDAR/ultrasonic sensor"""
    global front_distance
    num_ranges = len(scan_data.ranges)

    # Index for straight ahead (0° relative)
    zero_index = int((0.0 - scan_data.angle_min) / scan_data.angle_increment)

    # Small window in front (±5°)
    window = int((5.0 * 3.14159/180.0) / scan_data.angle_increment)
    start = max(0, zero_index - window)
    end   = min(num_ranges - 1, zero_index + window)

    front_ranges = [
        r for r in scan_data.ranges[start:end]
        if 0.05 < r < float('inf')
    ]
    if front_ranges:
        front_distance = min(front_ranges)
    else:
        front_distance = float('inf')

def main():
    global front_distance

    rospy.init_node("jetauto_mini_avoidance", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    rate = rospy.Rate(10)  # 10 Hz
    twist = Twist()

    while not rospy.is_shutdown():
        if front_distance < 0.3:  # Obstacle within 30 cm
            rospy.logwarn(f"Obstacle at {front_distance:.2f} m → Turning")
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # rotate left
        else:
            rospy.loginfo("Path clear → Moving forward")
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        pub.publish(twist)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
