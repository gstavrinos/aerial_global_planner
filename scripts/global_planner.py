#!/usr/bin/env python
import rospy
import traceback
from nav_msgs.msg import Path
from tf import TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from tf_pose_estimator.msg import Velocity
from tf_pose_estimator.msg import PosesAndVelocities

tf_ = None
path_pub = None
robot_pose = None

def init():
    global path_pub, tf_
    rospy.init_node('aerial_global_planner')
    # TODO add drone specific parameters here
    #max_velocity_x = rospy.get_param("~max_velocity_x", 1)
    rospy.Subscriber("tf_pose_estimator/poses_velocities", PosesAndVelocities, p_v_callback)
    tf_ = TransformListener()
    rospy.Subscriber("tf", TFMessage, tf_callback)
    path_pub = rospy.Publisher("aerial_global_planner/plan", Path, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()

def tf_callback(tf2):
    global robot_pose, tf_
    try:
        t = tf_.getLatestCommonTime("/odom", '/base_link')
        position, quaternion = tf_.lookupTransform("/odom", '/base_link', t)
        # Untested from here
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = '/base_link'
        ps.pose.position.x = position[0]
        ps.pose.position.y = position[1]
        ps.pose.position.z = position[2]
        ps.pose.orientation.x = quaternion[0]
        ps.pose.orientation.y = quaternion[1]
        ps.pose.orientation.z = quaternion[2]
        ps.pose.orientation.w = quaternion[3]
        robot_pose = ps
    except Exception as e:
        print traceback.format_exc()

def p_v_callback(pvmsg):
    global path_pub, robot_pose
    if robot_pose != None:
        path_msg = Path()
        path_msg.header.frame_id = 'base_link'
        path_msg.poses.append(pvmsg.latest_poses[-1])
        path_msg.poses.append(robot_pose)
        path_pub.publish(path_msg)

if __name__ == '__main__':
    init() 