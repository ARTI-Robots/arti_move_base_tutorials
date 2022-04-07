#!/usr/bin/env python
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from turtlesim.msg import Pose as TurtlePose


class OdomRepublisher(object):
    def __init__(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.pose_sub = rospy.Subscriber('turtle_pose', TurtlePose, self.sub_pose)

    def sub_pose(self, msg):
        """
        :type msg: TurtlePose
        """
        # rospy.loginfo(msg)
        current_time = rospy.Time.now()

        odom = Odometry(header=Header(stamp=current_time, frame_id='odom'), child_frame_id='base_link')
        odom.twist.twist = Twist(linear=Vector3(msg.linear_velocity, 0, 0),
                                 angular=Vector3(0, 0, msg.angular_velocity))
        odom.pose.pose = Pose(position=Point(msg.x, msg.y, 0),
                              orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, msg.theta)))

        self.odom_pub.publish(odom)
        self.tf_broadcaster.sendTransformMessage(TransformStamped(
            header=odom.header, child_frame_id=odom.child_frame_id,
            transform=Transform(translation=odom.pose.pose.position, rotation=odom.pose.pose.orientation)))


if __name__ == '__main__':
    rospy.init_node('odom_republisher')
    rospy.loginfo('starting turtlesim odom/tf publisher')
    OdomRepublisher()
    rospy.spin()
