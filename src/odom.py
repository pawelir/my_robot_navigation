#! /usr/bin/env python3

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf import TransformBroadcaster
import rospy
import tf

'''
Custom odometry publisher, based on the model state from gazebo
This publisher is not used anywhere, exists only for veryfications and tests
'''

rospy.init_node('custom_odometry_node')
odom_pub = rospy.Publisher('my_odom', Odometry)
rospy.wait_for_service('/gazebo/get_model_state')       # wait for service
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)        #connect to service

broadcaster = TransformBroadcaster()
odom  = Odometry()
header = Header()
header.frame_id = 'odom'

model = GetModelStateRequest()
model.model_name = 'my_robot'

r = rospy.Rate(5)

while not rospy.is_shutdown():
    
    result = get_model_srv(model)
    print(result)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)
    broadcaster.sendTransform((result.pose.position.x, result.pose.position.y, 0), tf.transformations.quaternion_from_euler(0, 0, result.pose.orientation.z), rospy.Time.now(), 'base_footprint', 'odom')

    r.sleep()


