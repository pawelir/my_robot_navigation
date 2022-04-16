#! /usr/bin/env python3

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf import TransformBroadcaster
import rospy
import tf

'''
Custom odometry publisher, based on the model state from gazebo
This publisher is not used anywhere, exists only for veryfications and tests
'''

rospy.init_node('simulated_odometry')
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
odom_pub = rospy.Publisher('my_odom', Odometry, queue_size=1)

broadcaster = TransformBroadcaster()
odom  = Odometry()
header = Header()
header.frame_id = 'odom'

model = GetModelStateRequest()
model.model_name = 'my_robot'

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)
    broadcaster.sendTransform((result.pose.position.x, result.pose.position.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, result.pose.orientation.z),
            rospy.Time.now(),
            'base_footprint',
            'odom')
    rate.sleep()
