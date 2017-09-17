#!/usr/bin/env python
import tf
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
max_velocity = 1 # m/s
robot_pose = None

def init():
    global path_pub, tf_, max_velocity
    rospy.init_node('aerial_global_planner')
    # TODO add drone specific parameters here
    max_velocity = rospy.get_param("~max_velocity", 5)
    rospy.Subscriber("tf_velocity_estimator/poses_velocities", PosesAndVelocities, p_v_callback)
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
    global path_pub, robot_pose, max_velocity
    if robot_pose != None:
        path_msg = Path()
        #path_msg.header.frame_id = '/base_link'
        path_msg.header.frame_id = '/odom'
        path_msg.header.stamp = rospy.Time.now()
        '''pvmsg.latest_poses[-1].header.frame_id = '/base_link'
        robot_quat = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]
        latest_pose = pvmsg.latest_poses[-1]
        latest_pose_quat = [latest_pose.pose.orientation.x, latest_pose.pose.orientation.y, latest_pose.pose.orientation.z, latest_pose.pose.orientation.w]
        robot_euler = tf.transformations.euler_from_quaternion(robot_quat)
        latest_pose_euler = tf.transformations.euler_from_quaternion(latest_pose_quat)
        latest_pose_euler = list(latest_pose_euler)
        latest_pose_euler[0] -= robot_euler[0]
        latest_pose_euler[1] -= robot_euler[1]
        latest_pose_euler[2] -= robot_euler[2]
        pvmsg.latest_poses[-1].pose.position.x += robot_pose.pose.position.x
        pvmsg.latest_poses[-1].pose.position.y += robot_pose.pose.position.y
        pvmsg.latest_poses[-1].pose.position.z += robot_pose.pose.position.z
        latest_pose_quat = tf.transformations.quaternion_from_euler(latest_pose_euler[0], latest_pose_euler[1], latest_pose_euler[2])
        pvmsg.latest_poses[-1].pose.orientation.x = latest_pose_quat[0]
        pvmsg.latest_poses[-1].pose.orientation.y = latest_pose_quat[1]
        pvmsg.latest_poses[-1].pose.orientation.z = latest_pose_quat[2]
        pvmsg.latest_poses[-1].pose.orientation.w = latest_pose_quat[3]
        '''

        # UBER TEST CODE STARTS HERE
        latest_pose = pvmsg.latest_poses[-1]
        lastx = latest_pose.pose.position.x
        lasty = latest_pose.pose.position.y
        latest_vel = pvmsg.latest_velocities[-1]
        lastvx = latest_vel.vx
        lastvy = latest_vel.vy
        t_ = (rospy.Time.now().to_sec() - latest_pose.header.stamp.to_sec())

        goalx , goaly = rendezvous(t_, lastx, lasty, lastvx, lastvy, robot_pose.pose.position.x, robot_pose.pose.position.y, max_velocity)

        # AND ENDS HERE!

        #path_msg.poses.append(pvmsg.latest_poses[-1])
        if goalx != None:
            latest_pose.pose.position.x = goalx
            latest_pose.pose.position.y = goaly
            path_msg.poses.append(latest_pose)
            path_msg.poses.append(robot_pose)
            path_pub.publish(path_msg)

# UNTESTED FUNCTION
def rendezvous(t_, helix, heliy, helivx, helivy, robotx, roboty, maxrobotv):
    goalx = None
    goaly = None
    for t in float_range(t_, 20, 0.1):
        # Helipad position after time = t
        hx = helix + (helivx * t)
        hy = heliy + (helivy * t)
        # Robot needed velocity to reach helipad's position
        neededvx = (robotx - hx) / t
        neededvy = (roboty - hy) / t
        if neededvx < max_velocity / 4 and neededvy < max_velocity / 4:
            goalx = hx
            goaly = hy
            break
        # TODO
    # Implement minimum distance point of rendezvous here
    return goalx, goaly

def lookAt(curr, goal):
    # TODO
    quat = None
    return quat

# UNTESTED FUNCTION
def float_range(x, y, jump):
    while x < y:
        yield x
        x += jump

if __name__ == '__main__':
    init()