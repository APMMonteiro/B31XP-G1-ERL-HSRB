#! /usr/bin/env python

print("Starting semantic_getter_srv.py")

import rospy
from custom_srvs.srv import StringSrv, StringSrvResponse
import json

class semantic_getter_srv():
    def __init__(self):
        rospy.loginfo("Initialising semantic_getter_srv")
        rospy.init_node('azm_semantic_getter_srv_node')
        self.service = rospy.Service('azm_semantic_getter_srv', StringSrv, self.get_obj)
        self.semantic_map_path = r'/workspace/src/nav/nav_tests/maps/semantic.txt'
    
    def load_semantic_map(self):
        try:
            with open(self.semantic_map_path, "r") as f:
                rospy.logdebug("Map loaded from {}".format(self.semantic_map_path))
                return json.loads(f.read())
        except Exception as e:
            rospy.logwarn("An error occured while loading the JSON semantic map: {}".format(e))

    def get_obj(self, object):
        response = StringSrvResponse()
        
        semantic_map = self.load_semantic_map()

        out = []
        for entry in semantic_map:
            if entry["name"] == object.data:
                out += [entry]

        response.response = json.dumps(out)
        return response

if __name__ == '__main__':
    print("executing semantic_getter_srv.py as main")
    print("Creating semantic_getter_srv obj")
    controller_main = semantic_getter_srv()
    print("semantic_getter_srv.py is spinning")
    rospy.spin()