#! /usr/bin/env python

print("Starting service_test.py")

import rospy
from std_msgs.msg import String

rospy.init_node('test_service_client')

#maybe should wait for service to be up with 
#rospy.wait_for_service('/service')

#connect to service
test_service = rospy.ServiceProxy('/azm_semantic_getter_server', String)


#make service message to send
request = String()
request.data = "bottle"

#make the request and receive output
output = test_service(request)
