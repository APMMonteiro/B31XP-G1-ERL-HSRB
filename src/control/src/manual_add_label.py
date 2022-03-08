#! /usr/bin/env python

print("Starting manual_add_label.py")

import rospy
from std_msgs.msg import String
import sys
sys.path.insert(1, 'C:\Users\AZM\Documents\Uni\Year 5 Semester 2\ERL\tmc_wrs_docker\src')
from nav.nav_tests.src.azmutils import dynamic_euclid_dist, str_to_obj, obj_to_str
#TODO import odometry msg

class manual_add_label():
    def __init__(self):
        # Base node inits
        rospy.loginfo("Initiating manual_add_label")
        rospy.init_node('manual_add_label')
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        # Goal publishing inits
        self.addLabel = rospy.Publisher('/azm_nav/semantic_label_additions', String, queue_size=1)
        self.sub = rospy.Subscriber('/azm_nav/semantic_manual_add', String, self.cb)
        self.semantic_goal = rospy.Publisher('/azm_nav/semantic_goal_listener', String, queue_size=1)
        self.goal_sub = rospy.Subscriber('/azm_nav/goal_result', String, self.goal_cb)
        self.reached = True
        self.odom = Odometry # FIXME
        self.odom_sub = rospy.Subscriber('/odometry', Odometry, self.odom_cb) # TODO FIXME

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
        _msg.data = obj_to_str(_label)
        self.publish_once(self.addLabel, _msg)

    def odom_cb(self, msg):
        self.odom = msg

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

if __name__ == '__main__':
    print("executing manual_add_label.py as main")
    print("Creating manual_add_label obj")
    manual_add_label = manual_add_label()
    manual_add_label.do_nav_example()
    print("manual_add_label.py is spinning")
    rospy.spin()
