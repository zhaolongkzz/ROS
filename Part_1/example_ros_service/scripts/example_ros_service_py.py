#!/usr/bin/env python2.7

import rospy
from example_ros_service.srv import *

# class Response:
#     def __init__(self, age, good_guy, on_the_list, nickname):
#         self.age = age
#         self.good_guy = good_guy
#         self.on_the_list = on_the_list
#         self.nickname = nickname

def handle(req):
    rospy.loginfo("callback activated")
    in_name = req.name
    print "in_name:" + in_name
    on_the_list = False
    if in_name == "Bob":
        rospy.loginfo("asked about Bob")
        # response1 = Response(32, False, True, "Bob the Terrible")
        return ExampleServiceMsgResponse(age = 32, good_guy = False, on_the_list = True, nickname = "Bob the Terrible") 

    if in_name == "Ted":
        rospy.loginfo("asked about Ted")
        # response2 = Response(21, True, True, "Bob the Terrible")
        return ExampleServiceMsgResponse(age = 21,good_guy = True,on_the_list = True,nickname = "Ted the Benevolent")
 

def example_ros_service_py():
    rospy.init_node('example_ros_service_py')
    service_py = rospy.Service('lookup_by_name', ExampleServiceMsg, handle)
    rospy.loginfo("Ready to look up names.") 
    rospy.spin()


if __name__ == "__main__":
    example_ros_service_py()