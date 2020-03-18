#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

def pubstringn( topic, msg, repeat, hz ):
  # initialize ROS node, setup String publisher on topic, get publish rate
  rospy.init_node( 'repeater', anonymous = True )
  pub = rospy.Publisher( topic, String, queue_size = 10 )
  rate = rospy.Rate( hz )

  # publish message repeat times at rate hz
  count = 0
  while count < repeat and not rospy.is_shutdown():
    rospy.loginfo( 'publishing %d of %d', count, repeat )
    count += 1
    pub.publish( msg )
    rate.sleep()

if __name__ == '__main__':
  if len(sys.argv) != 5:
    print( "usage: pubstringn.py topic message repeat hertz" )
    sys.exit( 1 )
  try:
    topic,msg,repeat,hz = sys.argv[1:]
    pubstringn( topic, msg, int( repeat ), int( hz ) )
  except rospy.ROSInterruptException:
    pass
