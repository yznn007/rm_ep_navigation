#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


class CmdVelRemap:
    def __init__(self):
        rospy.init_node("cmd_vel_remap")

        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel_rm_ep")

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self._callback, queue_size=1)

        rospy.loginfo("cmd_vel_remap: %s -> %s (x↔y swap)", self.input_topic, self.output_topic)

    def _callback(self, msg):
        out = Twist()
        out.linear.x = msg.linear.y
        out.linear.y = msg.linear.x
        out.linear.z = msg.linear.z
        out.angular = msg.angular
        self.pub.publish(out)


if __name__ == "__main__":
    try:
        node = CmdVelRemap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
