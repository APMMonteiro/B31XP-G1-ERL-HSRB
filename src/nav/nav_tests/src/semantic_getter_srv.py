#! /usr/bin/env python

print("Starting semantic_getter_srv.py")

import rospy
from std_msgs.msg import String

class semantic_getter_srv():
    def __init__(self):
        rospy.loginfo("Initialising semantic_getter_srv")
        rospy.init_node('azm_semantic_getter_srv')
        self.service = rospy.Service('azm_semantic_getter_srv', String, self.get_obj)
    
    def get_obj(self, object):
        print(object.data)
        response = String()
        response.data = "stuff"
        return response

if __name__ == '__main__':
    print("executing semantic_getter_srv.py as main")
    print("Creating semantic_getter_srv obj")
    controller_main = semantic_getter_srv()
    print("semantic_getter_srv.py is spinning")
    rospy.spin()