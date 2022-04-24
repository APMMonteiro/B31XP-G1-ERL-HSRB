#! /usr/bin/env python

print("Running semantic_getter_service.py")

import rospy
from custom_srvs.srv import StringSrv, StringSrvResponse
import json

class SemanticGetterService():
    def __init__(self):
        rospy.init_node('azm_nav_semantic_getter_srv_node')
        rospy.loginfo("Initialised semantic_getter_srv")
        self.service = rospy.Service('/azm/nav/semantic/get_objects', StringSrv, self.get_obj)
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
    print("Executing semantic_getter_srv.py as main")
    print("Creating SemanticGetterService obj")
    semantic_getter_service = SemanticGetterService()
    rospy.loginfo("SemanticGetterService obj is spinning")
    rospy.spin()