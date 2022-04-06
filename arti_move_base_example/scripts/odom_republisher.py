#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from turtlesim.msg import Pose as turtlepose

#rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
base_link_broadcaster = tf.TransformBroadcaster()
#last_pub
#initialized;

class odom_creater(object):

  def __init__(self):
     self.initialized = False;

     self.last_pub = rospy.Time.now()
     rospy.Subscriber("/turtle1/pose", turtlepose, self.sub_pose)

     #rospy.spin()

  def sub_pose(self, msg):
     #rospy.loginfo(msg)
     current_time = rospy.Time.now()
     if self.initialized == False:
       self.odom = Odometry()
     self.odom.header.stamp = current_time
     self.odom.header.frame_id = "odom"
     self.odom.child_frame_id = "base_link"
     x = msg.x
     y = msg.y
     th = msg.theta
     odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
     v = msg.linear_velocity
     v_ang = msg.angular_velocity

     if self.initialized == True:
       dx = self.odom.pose.pose.position.x - x
       dy = self.odom.pose.pose.position.y - y
     #  if math.sqrt((dx*dx)+(dy*dy)) != 0:
     #    v = (current_time - self.last_pub).to_sec() / math.sqrt((dx*dx)+(dy*dy))
     #  else:
     #    v = 0.001
     else:
       self.initialized = True

     last_pub = rospy.Time.now()
     self.odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, v_ang))
     self.odom.pose.pose = Pose(Point(x,y,0), Quaternion(*odom_quat))
     
     odom_pub.publish(self.odom)

     base_link_broadcaster.sendTransform((x,y,0), 
					 tf.transformations.quaternion_from_euler(0, 0, th),
					 current_time,
					 "base_link",
					 "odom")

  def start(self):
    rospy.loginfo("starting Turtlebot odom/tf publisher")

    while not rospy.is_shutdown():
      rospy.spin()

if __name__ == '__main__':
     rospy.init_node('turtlesim_odom', anonymous=False)
     odom_node = odom_creater()
     odom_node.start()
