#! /usr/bin/env python

print("Running manual_add_label.py")

import rospy
from std_msgs.msg import String
import json

class ManualAddLabel():
    def __init__(self):
        # Base node inits
        rospy.init_node('manual_add_label_node')
        rospy.loginfo("Initiated manual_add_label_node")
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        # Goal publishing inits
        self.addLabel = rospy.Publisher('/azm/ctrl/semantic/label_additions', String, queue_size=1)
        self.sub = rospy.Subscriber('/azm/nav/semantic/manual_add', String, self.cb)
        # self.odom = Odometry # FIXME
        # self.odom_sub = rospy.Subscriber('/odometry', Odometry, self.odom_cb) # TODO FIXME

    def publish_once(self, topic, msg, content="message"):
        rospy.loginfo("Attempting to publish {} to {}".format(content, topic.name))
        while not self.ctrl_c:
            connections = topic.get_num_connections()
            if connections > 0:
                topic.publish(msg)
                rospy.loginfo("Message published to {}".format(topic.name))
                break
            else:
                #rospy.loginfo("No subscribers on {}, sleeping.".format(topic.name))
                pass

    def cb(self, msg):
        _label = {
            "name":msg.data,
            "type":"test",
            "coords":[
                1,
                2,
                3
            ],
            "others":{}
            }
        _msg = String()
        _msg.data = json.dumps(_label)
        self.publish_once(self.addLabel, _msg)

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

if __name__ == '__main__':
    print("Executing manual_add_label.py as main")
    print("Creating ManualAddLabel obj")
    manual_add_label = ManualAddLabel()
    rospy.loginfo("ManualAddLabel obj is spinning")
    rospy.spin()
