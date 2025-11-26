#!/usr/bin/env python3
import rospy, sys
from geometry_msgs.msg import Twist

def move_robot(direction, duration):
    rospy.init_node('jetauto_move_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    twist = Twist()

    if direction == "forward":
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.angular.z = 0.0
    elif direction == "backward":
        twist.linear.x = -0.2
        twist.linear.y = 0.0
        twist.angular.z = 0.0
    elif direction == "turnleft":
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.5
    elif direction == "turnright":
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = -0.5
    elif direction == "shiftleft":
        twist.linear.x = 0.0
        twist.linear.y = 0.2
        twist.angular.z = 0.0
    elif direction == "shiftright":
        twist.linear.x = 0.0
        twist.linear.y = -0.2
        twist.angular.z = 0.0
    else:
        rospy.logerr("Invalid direction! Use forward | backward | turnleft | turnright | shiftleft | shiftright")
        return

    rospy.loginfo(f"Moving {direction} for {duration} seconds...")
    ticks = int(duration * 10)
    for _ in range(ticks):  # 2 seconds
        pub.publish(twist)
        rate.sleep()

    # stop
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.loginfo("Stopped.")

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print("Usage: rosrun jetauto_movement move.py <direction> [duration]")
            sys.exit(1)

        direction = sys.argv[1].lower()
        duration = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        move_robot(direction, duration)
    except rospy.ROSInterruptException:
        pass
