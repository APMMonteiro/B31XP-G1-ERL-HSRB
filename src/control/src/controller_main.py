#! /usr/bin/env python

print("Running controller_main.py")

import rospy
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry
from math import pi
import json
import tf
import tf2_ros
import sys
from custom_srvs.srv import StringSrv, StringSrvResponse, StringSrvRequest
from nav_tests.msg import CoordGoalAction, CoordGoalGoal
import actionlib
# sys.path.insert(1, 'C:\Users\AZM\Documents\Uni\Year 5 Semester 2\ERL\tmc_wrs_docker\src')
# from nav.nav_tests.src.azmutils import dynamic_euclid_dist, str_to_obj, obj_to_str

def euler_from_quaternion(q):
    q = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w], 'rxyz')
    return (q[0]/pi * 180, q[1]/pi * 180, q[2]/pi * 180)


class ControllerMain():
    def __init__(self):
        # Base node inits
        rospy.init_node('azm_ctrl_main_node')
        rospy.loginfo("Initiated azm_ctrl_main_node")
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)

        self.fetch_sub = rospy.Subscriber('/azm/ctrl/fetch_this', String, self.main)
        
        # connect to the other nodes' topics
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.curr_odom = Odometry()
        self.semantic_goal_pub = rospy.Publisher('/azm/nav/semantic/goal_listener', String, queue_size=1)
        self.coord_goal_pub = rospy.Publisher('/azm/nav/coord_goal_listener', Float64MultiArray, queue_size=1)
        self.semantic_goal_result_sub = rospy.Subscriber('/azm/nav/goal_result', String, self.sem_goal_cb)
        self.semantic_goal_result_flag = False
        self.semantic_goal_result = String()
        # Get pose inits
        #self.pose_sub = rospy.Subscriber('/tf', TFMessage, self.pose_cb)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        # connect to service and action servers
        rospy.logdebug("Connecting to services and action servers")
        self.semantic_getter_service = rospy.ServiceProxy('/azm/nav/semantic/get_objects', StringSrv)
        self.semantic_getter_service.wait_for_service()
        rospy.logdebug("Controller connected to service: '/azm/nav/semantic/get_objects'")
        self.coord_goal_action_client = actionlib.SimpleActionClient("/azm/nav/coord_goal_action_server", CoordGoalAction)
        self.coord_goal_action_client.wait_for_server()
        rospy.logdebug("Controller connected to action: '/azm/nav/coord_goal_action_server'")
        

    def publish_once(self, topic, msg, content="message"):
        attempts = 8
        time_multiplier = 1.5
        sleep = 0.2
        rospy.loginfo("Attempting to publish {} to {}".format(content, topic.name))
        while not self.ctrl_c and attempts:
            connections = topic.get_num_connections()
            if not attempts:
                rospy.logwarn("No listeners on {}, {} wasn't sent.".format(topic, content))
                return
            if connections > 0:
                topic.publish(msg)
                rospy.loginfo("Message published to {}".format(topic.name))
                break
            else:
                rospy.loginfo("No subscribers on {}, sleeping.".format(topic.name))
                rospy.sleep(sleep)
                attempts -= 1
                sleep *= time_multiplier

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def odom_cb(self, msg):
        self.curr_odom = msg

    def sem_goal_cb(self, msg):
        self.semantic_goal_result_flag = True
        self.semantic_goal_result = msg

    def get_pose(self, one, two):
        try:
            rospy.loginfo("Attempting to get transform from {} to {}".format(one, two))
            #(trans,rot) = self.tfListener.lookup_transform(one, two, rospy.Time(0), rospy.Duration(4))
            transform = self.tfBuffer.lookup_transform(one, two, rospy.Time(0), rospy.Duration(1))
            rospy.logdebug("{}, {}".format(transform.transform.translation.x, transform.transform.translation.y))
            rospy.logdebug("rotation:{}".format(euler_from_quaternion(transform.transform.rotation)))
            _pose = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                euler_from_quaternion(transform.transform.rotation)[2]]
            return _pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error when getting pose from {} to {}: {}".format(one, two, e))
    
    def main(self, fetch_object):
        # main code to execute the task

        # ask semantic map if the requested object exists, if not just exit
        request = StringSrvRequest()
        request.data = fetch_object.data
        response = self.semantic_getter_service(request)
        requested_objects = json.loads(response.response)
        if len(requested_objects) == 0:
            rospy.logwarn("Semantic map does not have {} objects".format(fetch_object.data))
            return 1
        
        # save current location, it is the location we'll return to
        start_loc = self.get_pose("map", "base_link")

        # if it does tell it to go to it and await a success
        # maybe make this navigation into an action server
        # reset used flags
        self.semantic_goal_result.data = ""
        self.semantic_goal_result_flag = False
        self.publish_once(self.semantic_goal_pub, fetch_object , "fetch_object: " + fetch_object.data)
        rospy.loginfo("Waiting to move to goal location")
        while not self.semantic_goal_result_flag:
            rospy.sleep(.5)
            sys.stdout.write(".")
            sys.stdout.flush()
        print("")
        if self.semantic_goal_result.data != "success":
            rospy.loginfo("Failed to reach goal, stopping fetch task")
            return
        
        # if this was a success it means we're at the object's location
        # so tell the manipulation to work and await success, is this an action?
        rospy.loginfo("Reached object location, passing to manipulation (but not really)")
        
        # upon successful grasp of the object return to the location the request was made
        rospy.loginfo("Got object returning to granny")
        # self.semantic_goal_result.data = ""
        # self.semantic_goal_result_flag = False
        # _coord = Float64MultiArray()
        # _coord.data = [start_loc[0], start_loc[1], start_loc[2]]
        # self.publish_once(self.coord_goal_pub, _coord, "return location")
        # rospy.loginfo("Waiting to move to goal location.")
        # while not self.semantic_goal_result_flag:
        #     rospy.sleep(.5)
        #     sys.stdout.write(".")
        #     sys.stdout.flush()
        # print("")
        # if self.semantic_goal_result.data != "success":
        #     rospy.loginfo("Failed to reach goal, stopping fetch task")
        #     return
        
        _coords = CoordGoalGoal()
        _coords.goal = [start_loc[0], start_loc[1], start_loc[2]]
        self.coord_goal_action_client.send_goal(_coords)
        rospy.loginfo("Waiting to move to goal location.")
        while self.coord_goal_action_client.get_state() < 2:
            rospy.sleep(.5)
            sys.stdout.write(".")
            sys.stdout.flush()
        sys.stdout.write("\n")
        rospy.loginfo("action state is: ")
        rospy.loginfo(self.coord_goal_action_client.get_state())
        result = self.coord_goal_action_client.get_result()
        if result.result != "success":
            rospy.loginfo("Failed to reach goal, stopping fetch task")
            return
        # hand in the object?
        # idle / execution over
        rospy.loginfo("Finished task")

        

if __name__ == '__main__':
    print("Executing controller_main.py as main")
    print("Creating ControllerMain obj")
    controller_main = ControllerMain()
    print("ControllerMain obj is spinning")
    rospy.spin()
