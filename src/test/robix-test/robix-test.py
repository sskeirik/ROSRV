#!/usr/bin/python

import rospy
from std_msgs.msg import String
from marti_common_msgs.msg import Float32Stamped
import std_msgs.msg

def talker():
    pub = rospy.Publisher('/rv/monitored/aa_quant_monitor_speed',
            Float32Stamped, queue_size=10)
    rospy.init_node('talker', anonymous=False)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        pub.publish(Float32Stamped(h, 10.4))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

