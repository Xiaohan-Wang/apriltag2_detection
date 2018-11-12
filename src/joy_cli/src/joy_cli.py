#!/usr/bin/env python
# license removed for brevity
import rospy
import sys

from sensor_msgs.msg import Joy

from __builtin__ import True


def keyCatcher(host):
    pub = rospy.Publisher('/'+host+'/joy', Joy, queue_size=1)
    
    while not rospy.is_shutdown():
        direction = raw_input('Enter direction(a,w,s,d)--> ')
        if direction == 'w':
            axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif direction == 's':
            axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif direction == 'd':
            axes = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        elif direction == 'a':
            axes = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        else:
            axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        msg = Joy(header=None, axes=axes, buttons=None)
        pub.publish(msg)
        rospy.sleep(0.5)

        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = Joy(header=None, axes=axes, buttons=None)
        pub.publish(msg)

if __name__ == '__main__':

    rospy.init_node("joy_cli")
    print "Starting node %s" % rospy.get_name()

    if len(sys.argv) != 2:
        print "Vehicle name not passed as a command line argument, expected to be passed as ROS variable"
        try:
      	    hostname = rospy.get_param("~veh")
        except:
    	    raise Exception("ROS parameter '~veh' not found!")
    else:
        hostname = sys.argv[1]

    print 'Hostname: %s' % hostname



    try:
        keyCatcher(host = hostname)
    except rospy.ROSInterruptException:
	    raise Exception("Error encountered when attempting to start joy-cli keyCatcher!")
