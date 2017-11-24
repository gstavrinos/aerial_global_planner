#!/usr/bin/env python
import tf
import math
import rospy
import traceback
from nav_msgs.msg import Path
from tf import TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from tf_velocity_estimator.msg import Velocity
from tf_velocity_estimator.msg import PosesAndVelocities

tf_ = None
path_pub = None
max_velocity = 1 # m/s
robot_pose = None
tf_broadcaster = None

def init():
    global path_pub, tf_, max_velocity, tf_broadcaster
    rospy.init_node('aerial_global_planner')
    # TODO add drone specific parameters here
    max_velocity = rospy.get_param("~max_velocity", 5)
    rospy.Subscriber("tf_velocity_estimator/poses_velocities", PosesAndVelocities, p_v_callback)
    tf_ = TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
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
        # TODO subscribe to odom instead of tf! wtf!!
    except Exception as e:
        #print traceback.format_exc()
        pass

def p_v_callback(pvmsg):
    global path_pub, robot_pose, max_velocity, tf_broadcaster
    if robot_pose != None:
        path_msg = Path()
        path_msg.header.frame_id = '/odom'
        path_msg.header.stamp = rospy.Time.now()

        # UBER TEST CODE STARTS HERE
        latest_pose = pvmsg.latest_poses[-1]
        lastx = latest_pose.pose.position.x
        lasty = latest_pose.pose.position.y
        lastz = latest_pose.pose.position.z
        sumvx = 0
        sumvy = 0
        for v in pvmsg.latest_velocities:
            sumvx += v.vx
            sumvy += v.vy
        lastvx = sumvx / len(pvmsg.latest_velocities)
        lastvy = sumvy / len(pvmsg.latest_velocities)
        latest_vel = pvmsg.latest_velocities[-1]
        print lastvx
        #lastvx = latest_vel.vx
        #lastvy = latest_vel.vy
        t_ = (rospy.Time.now().to_sec() - latest_pose.header.stamp.to_sec())

        goalx , goaly, goalz, yaw = rendezvous(t_, lastx, lasty, lastz, lastvx, lastvy, robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z, max_velocity)

        # AND ENDS HERE!

        #path_msg.poses.append(pvmsg.latest_poses[-1])
        if goalx != None:
            latest_pose.pose.position.x = goalx
            latest_pose.pose.position.y = goaly
            latest_pose.pose.position.z = goalz
            path_msg.poses.append(latest_pose)
            path_msg.poses.append(robot_pose)
            path_pub.publish(path_msg)

            goal_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            tf_broadcaster.sendTransform(
                (goalx, goaly, goalz),
                goal_quat,
                rospy.Time.now(),
                'robot_goal',
                'odom')

# UNTESTED FUNCTION
def rendezvous(t_, helix, heliy, heliz, helivx, helivy, robotx, roboty, robotz, maxrobotv):
    global tf_broadcaster
    goalx = None
    goaly = None
    goalz = None
    #TODO z is missing
    for t in float_range(t_, 20, 0.1):
        # Helipad position after time = t
        hx = helix + (helivx * t)
        hy = heliy + (helivy * t)
        if t == t_:
            tf_broadcaster.sendTransform(
                (hx, hy, heliz),
                (0, 0, 0, 1),
                rospy.Time.now(),
                'goal_prediction',
                'odom')
        # Robot needed velocity to reach helipad's position
        neededvx = (robotx - hx) / t
        neededvy = (roboty - hy) / t
        neededvz = (robotz - heliz) / t
        if abs(neededvx) < maxrobotv / 4 and abs(neededvy) < maxrobotv / 4 and abs(neededvz) < maxrobotv / 4:
            goalx = hx
            goaly = hy
            goalz = heliz
            break
        # TODO
    yaw = 0.0
    if goalx!= None:
        yaw = lookAt(goalx, goaly, helix, heliy)
    return goalx, goaly, goalz, yaw

def lookAt(goalx, goaly, helix, heliy):
    dx = helix - goalx
    dy = heliy - goaly
    return math.atan2(dy,dx)

def float_range(x, y, jump):
    while x < y:
        yield x
        x += jump

if __name__ == '__main__':
    init()