#!/usr/bin/env python2

import rospy as ros
from subprocess import Popen, check_call
from std_msgs.msg import Float32, Header
from marti_common_msgs.msg import Float32Stamped
from test_without_rvmaster import RosTestPacket, ros_init

import pytest

class Float32Packet(RosTestPacket):

    def __init__(self, topic_name, to_send, prefix):
        super(Float32Packet, self).__init__(topic_name, Float32, 'std_msgs/Float32', to_send, prefix)

    def to_formatted_string(self, msg):
        return str(msg.data)

    def stamp_msg(self, value):
        return Float32(value)

class Float32StampedPacket(RosTestPacket):

    def __init__(self, topic_name, to_send, prefix):
        super(Float32StampedPacket, self).__init__(topic_name, Float32Stamped,
                'marti_common_msgs/Float32Stamped', to_send, prefix)

    def to_formatted_string(self, msg):
        return str(msg.value)

    def stamp_msg(self, value):
        h = ros.Header()
        h.stamp = ros.Time.now()
        return Float32Stamped(h, float(value))


@pytest.fixture()
def launch_monitor(request):
    def run_launch_monitor(monitor_name):
        monitor = Popen(['/usr/bin/env', 'rosrun', 'rvmonitor', monitor_name])
        def cleanup():
            monitor.terminate()
        request.addfinalizer(cleanup)
    return run_launch_monitor

@pytest.fixture()
def dl_pipeline(ros_init):
    def run_dl_pipeline(test_packets):
        rate = ros.Rate(1); #Hz
        for packet in test_packets:
            # register callbacks and publishers
            ros.Subscriber(packet.topic_name, packet.msg_class, packet.callback)
            publisher = ros.Publisher(packet.send_topic(), packet.msg_class, queue_size=10)
            for unstamed_msg in packet.to_send:
                stamped_msg = packet.stamp_msg(unstamed_msg)
                # send stamped message
                publisher.publish(stamped_msg)
            rate.sleep()
    return run_dl_pipeline

prefix = '/rv/monitored'

def test_roscore__monitored_dl_watertank(dl_pipeline, launch_monitor):
   launch_monitor('monitor-dl-watertank')
   sensor_packets = [ Float32Packet( '/level_sensor', [x], '') for x in [0,5, 0.5] ]

   control_packets = [ Float32StampedPacket( '/flow_control_cmd', [x], prefix) for x in [0.0, 0.75] ]
   dl_pipeline( [ sensor_packets[0], control_packets[0]
                , sensor_packets[1], control_packets[1] ])

   # Unsafe Control Changed to safe control
   assert(control_packets[1].recieved[0] == '0.0')




