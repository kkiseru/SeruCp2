#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_data):
    # LaserScan.ranges is a list of distances (in meters)
    # Angle 0 is forward (depends on sensor mounting)
    # To be safe, we take a small window around the front (e.g. ±5°)
    
    num_ranges = len(scan_data.ranges)
    angle_min = scan_data.angle_min
    angle_max = scan_data.angle_max
    angle_increment = scan_data.angle_increment

    # Find the index for 0° (straight ahead)
    zero_index = int((0.0 - angle_min) / angle_increment)

    # Take ±5° around the center
    window_size = int((5.0 * 3.14159/180.0) / angle_increment)  # 5 degrees in radians
    start = max(0, zero_index - window_size)
    end = min(num_ranges - 1, zero_index + window_size)

    # Extract distances in that window
    front_ranges = scan_data.ranges[start:end]

    # Filter out invalid values (0.0 or inf)
    valid_ranges = [r for r in front_ranges if r > 0.01 and r < float('inf')]

    if valid_ranges:
        front_distance = min(valid_ranges)
        rospy.loginfo(f"Obstacle in front: {front_distance:.2f} m")
    else:
        rospy.loginfo("No valid obstacle detected in front.")

def main():
    rospy.init_node('obstacle_distance_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
