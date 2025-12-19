#!/usr/bin/env python3
"""
JetAuto Autonomous Delivery System
User selects delivery point (1-4), robot navigates there, then returns to home (point 5)
"""

import rospy
import json
import os
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class DeliverySystem:
    def __init__(self, waypoint_file=None):
        rospy.init_node('delivery_system')
        
        # Default waypoint file
        if waypoint_file is None:
            self.waypoint_file = os.path.expanduser(
                "~/Desktop/seruCP2/waypoint.json"
            )
        else:
            self.waypoint_file = waypoint_file
        
        # Move base client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # Load waypoints
        self.waypoints = self.load_waypoints()
        self.home_point = self.waypoints[4]  # Point 5 is home/return point
        
        rospy.loginfo("Delivery System initialized")
        rospy.loginfo(f"Home point: {self.home_point['name']}")
    
    def load_waypoints(self):
        """Load waypoints from JSON file"""
        try:
            with open(self.waypoint_file, 'r') as f:
                waypoints = json.load(f)
            rospy.loginfo(f"Loaded {len(waypoints)} waypoints")
            return waypoints
        except Exception as e:
            rospy.logerr(f"Failed to load waypoints: {e}")
            return []
    
    def send_goal(self, waypoint):
        """Send navigation goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = waypoint['x']
        goal.target_pose.pose.position.y = waypoint['y']
        goal.target_pose.pose.position.z = waypoint['z']
        
        # Default orientation (facing forward)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo(f"Sending goal to {waypoint['name']}")
        self.client.send_goal(goal)
        
        # Wait for result
        finished = self.client.wait_for_result()
        
        if finished:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"✓ Reached {waypoint['name']}")
                return True
            else:
                rospy.logwarn(f"✗ Failed to reach {waypoint['name']}")
                return False
        else:
            rospy.logerr("Goal cancelled or timed out")
            return False
    
    def delivery_mission(self, point_number):
        """Execute delivery: go to point -> return to home"""
        if point_number < 1 or point_number > 4:
            rospy.logerr("Invalid point number. Choose 1-4")
            return False
        
        delivery_point = self.waypoints[point_number - 1]
        
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"DELIVERY MISSION STARTED")
        rospy.loginfo(f"Destination: {delivery_point['name']}")
        rospy.loginfo("=" * 50)
        
        # Go to delivery point
        rospy.loginfo(f"\n[PHASE 1] Navigating to {delivery_point['name']}...")
        success = self.send_goal(delivery_point)
        
        if not success:
            rospy.logerr("Failed to reach delivery point")
            return False
        
        # Wait at delivery point
        rospy.loginfo(f"\n[PHASE 2] Arrived at {delivery_point['name']}")
        rospy.loginfo("Package delivered! Waiting 5 seconds...")
        rospy.sleep(5)
        
        # Return to home
        rospy.loginfo(f"\n[PHASE 3] Returning to {self.home_point['name']}...")
        success = self.send_goal(self.home_point)
        
        if success:
            rospy.loginfo("=" * 50)
            rospy.loginfo("✓ DELIVERY MISSION COMPLETED")
            rospy.loginfo("=" * 50)
            return True
        else:
            rospy.logerr("Failed to return to home")
            return False
    
    def print_menu(self):
        """Print delivery menu"""
        print("\n" + "=" * 50)
        print("JETAUTO AUTONOMOUS DELIVERY SYSTEM")
        print("=" * 50)
        print("\nAvailable Delivery Points:")
        for i in range(1, 5):
            wp = self.waypoints[i - 1]
            print(f"  {i}. {wp['name']:15} - X: {wp['x']:8.3f}, Y: {wp['y']:8.3f}")
        print(f"\n  H. HOME ({self.home_point['name']:15}) - X: {self.home_point['x']:8.3f}, Y: {self.home_point['y']:8.3f}")
        print(f"  Q. QUIT")
        print("=" * 50)
    
    def run_interactive(self):
        """Interactive menu for deliveries"""
        while not rospy.is_shutdown():
            self.print_menu()
            choice = input("\nSelect delivery point (1-4, H for home, Q to quit): ").strip().upper()
            
            if choice == 'Q':
                rospy.loginfo("Exiting delivery system")
                break
            elif choice == 'H':
                rospy.loginfo("Navigating to home point...")
                self.send_goal(self.home_point)
            elif choice in ['1', '2', '3', '4']:
                point_num = int(choice)
                self.delivery_mission(point_num)
            else:
                print("Invalid choice. Try again.")

if __name__ == '__main__':
    try:
        delivery_system = DeliverySystem()
        delivery_system.run_interactive()
    except rospy.ROSInterruptException:
        rospy.loginfo("Delivery system interrupted")
