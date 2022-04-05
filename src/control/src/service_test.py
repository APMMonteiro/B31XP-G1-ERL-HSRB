#! /usr/bin/env python

print("Starting service_test.py")

import rospy
from custom_srvs.srv import StringSrv, StringSrvRequest
from std_msgs.msg import String

rospy.init_node('test_service_client')

#maybe should wait for service to be up with 
#rospy.wait_for_service('/service')

#connect to service
test_service = rospy.ServiceProxy('/azm/nav/semantic/get_objects', StringSrv)


#make service message to send
request = StringSrvRequest()
request.data = "bottle"

#make the request and receive output
output = test_service("bottle")
print(output)
