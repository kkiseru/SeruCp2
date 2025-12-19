import rospy
import os
from std_srvs.srv import Empty, EmptyResponse

class MapSaveNode():
    def __init__(self, name):
        rospy.init_node(name, anonymous=False)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        goal_topic = rospy.get_param('~goal', 'move_base_simple/goal')
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)

        rospy.Service('/map_save', Empty, self.map_save_srv_callback)

    def map_save_srv_callback(self, msg):
        os.system('rosrun map_server map_saver -f $HOME/jetauto_ws/src/jetauto_slam/maps/explore')
        rospy.loginfo('************map saved as explore**************')
        return EmptyResponse()

if __name__ == '__main__':
    MapSaveNode('map_save')
