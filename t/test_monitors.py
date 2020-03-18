#!/usr/bin/env python2

import rospy as ros
from subprocess import Popen, check_call
from std_msgs.msg import String, ColorRGBA, Float32, Header
from marti_common_msgs.msg import Float32Stamped
import time as pytime

import pytest

# Base Test Packet Class
# ======================

class RosTestPacket(object):
    def __init__(self, topic_name, msg_class, msg_type, to_send, callback):
        self.topic_name = topic_name
        self.msg_class = msg_class
        self.msg_type = msg_type
        self.to_send = to_send
        self.callback = callback

    @classmethod
    def to_formatted_string(cls, msg):
        pass

    def stamp_msg(self, msg):
        return msg


# Derived Test Packets
# ====================

class StringPacket(RosTestPacket):
    def __init__(self, topic_name, to_send, callback):
        super(StringPacket, self).__init__( topic_name
                                          , String, 'std_msgs/String'
                                          , to_send, callback)

    @classmethod
    def to_formatted_string(cls, msg):
        return msg.data

# Fixtures
#=========

@pytest.fixture(scope='session')
def launch_master(request):
    def run_launch_master(monitor_topics_list):
        nested_list__topics = [ ['--monitor-topic',  topic]
                                    for topic in monitor_topics_list ]
        flattened_list__topics = [ elements
                                    for sub_list__topics in nested_list__topics
                                        for elements in sub_list__topics ]
        master = Popen( ['/usr/bin/env', 'rosrun', 'rvmaster', 'rvmaster']
                      + flattened_list__topics)
        def cleanup():
            master.terminate()
        request.addfinalizer(cleanup)
    return run_launch_master


@pytest.fixture(scope="session")
def session_init(get_options, launch_master):
    if 'with-rvmaster' in get_options['monitor_flags']:
        launch_master(get_options['monitor_topics_list'])
    # Sleep(1)
    pytime.sleep(1)
    ros.init_node('test', anonymous=True)

@pytest.fixture()
def simple_pipeline(request, session_init, get_options):
    def run_simple_pipeline(test_packets):
        registered_topics = []
        if('unmonitored_channel' in request.node.name):
            prefix = ''
        else:
            prefix = get_options['topic_prefix']
        rate = ros.Rate(10);
        for packet in test_packets:
            # register callbacks. Don't re-register new callbacks for existing topics
            if not packet.topic_name in registered_topics:
                ros.Subscriber(packet.topic_name, packet.msg_class, packet.callback.as_callable)
                registered_topics.append(packet.topic_name)
                rate.sleep()
            for unstamped_msg in packet.to_send:
                stamped_msg = packet.stamp_msg(unstamped_msg)
                check_call(['rostopic', 'pub', '--once'
                           , prefix + packet.topic_name
                           , packet.msg_type
                           , str(stamped_msg)])
            rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    return run_simple_pipeline

@pytest.fixture()
def launch_monitor(request, get_options):
    def run_launch_monitor(monitor_name):
        monitor = Popen(['/usr/bin/env', 'rosrun', 'rvmonitor', monitor_name, get_options['monitor_flags']])
        def cleanup():
            monitor.terminate()
        request.addfinalizer(cleanup)
        return True
    return run_launch_monitor

class CallBack(object):
    def __init__(self, to_formatted_string):
        self.recieved_list = []
        self.to_formatted_string = to_formatted_string

    def as_callable(self, msg):
        self.recieved_list.append(self.to_formatted_string(msg))


# Basic Tests
# ===========

def test_roscore__unmonitored_channel_pipeline(simple_pipeline):
    string_callback = CallBack(StringPacket.to_formatted_string)
    test_packets = [StringPacket('/unmonitored', ['Hi!'], string_callback)]
    simple_pipeline(test_packets)

    assert ['Hi!'] == string_callback.recieved_list

def test_roscore__monitored_channel__no_monitor_running(simple_pipeline):
    string_callback = CallBack(StringPacket.to_formatted_string)
    test_packets = [StringPacket('/chatter', ['Hi!', 'Hi!'], string_callback)]
    simple_pipeline(test_packets)

    assert([] == string_callback.recieved_list)

def test_roscore__monitored_channel(simple_pipeline, launch_monitor):
    launch_monitor('monitor-fixed-output')
    string_callback = CallBack(StringPacket.to_formatted_string)
    test_packets = [StringPacket('/chatter', ['Hi!', 'Hi!'], string_callback)]
    simple_pipeline(test_packets)

    assert(['B', 'B'] == string_callback.recieved_list)
