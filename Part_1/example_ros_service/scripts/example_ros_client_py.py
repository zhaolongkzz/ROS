#!/usr/bin/env python2.7

import sys, rospy
from example_ros_service.srv import *

def example_ros_client_py(res):
    rospy.wait_for_service('lookup_by_name')
    try:
        client_py = rospy.ServiceProxy('lookup_by_name', ExampleServiceMsg)
        resp = client_py(res)
        found_on_list = False
        while not rospy.is_shutdown():
            # in_name = input("enter a name (x to quit):")
            if res == "x":
                return 0
            # in_name = resp
            # if client.call():
            elif resp.on_the_list:
                print res + " is known as " + resp.nickname
                print "He is " + str(resp.age) + " years old"
                if resp.good_guy:
                    print "He is reported to be a good guy"
                    return 0
                else:
                    print "Avoid him; he is not a good guy"
                    return 0
            else:
                print res + " is not in my database"
                return 0

    except rospy.ServiceException, e:
        print "Service call failed : %s" % e 


def usage():
    print "enter a name (x to quit):"
    return "%s [name]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        in_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s" % in_name
    example_ros_client_py(in_name)