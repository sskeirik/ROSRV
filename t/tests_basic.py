#!/usr/bin/env python2

import rospy as ros
from subprocess import Popen, check_call
from std_msgs.msg import String, ColorRGBA
import time as pytime

import pytest

class RosTestPacket(object):
    def __init__(self, topic_name, msg_class, msg_type, to_send):
        self.topic_name = topic_name
        self.msg_class = msg_class
        self.msg_type = msg_type
        self.to_send = to_send
        self.recieved = []

    def to_formatted_string(self, msg):
        pass

    def callback(self, msg):
        self.recieved += [self.to_formatted_string(msg)]

class StringPacket(RosTestPacket):
    def __init__(self, topic_name, to_send):
        super(StringPacket, self).__init__(topic_name, String, 'std_msgs/String', to_send)

    def to_formatted_string(self, msg):
        return msg.data


class ColorRGBAPacket(RosTestPacket):
    def __init__(self, topic_name, to_send):
        super(ColorRGBAPacket, self).__init__(topic_name, ColorRGBA, 'std_msgs/ColorRGBA', to_send)

    def to_formatted_string(self, msg):
        return '({},{},{},{})'.format(msg.r, msg.g, msg.b, msg.a)

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
        if('unmonitored_channel' in request.node.name):
            prefix = ''
        else:
            prefix = get_options['topic_prefix']
        rate = ros.Rate(10);
        for packet in test_packets:
            # register callbacks
            ros.Subscriber(packet.topic_name, packet.msg_class, packet.callback)
            rate.sleep()
            for msg in packet.to_send:
                check_call(['rostopic', 'pub', '--once', prefix + packet.topic_name, packet.msg_type, msg])
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


# Tests
# =====

def test_roscore__unmonitored_channel_pipeline(simple_pipeline):
    test_packets = [StringPacket('/unmonitored', ['Hi!'])]
    simple_pipeline(test_packets)
    assert ['Hi!'] == test_packets[0].recieved

def test_roscore__monitored_channel__no_monitor_running(simple_pipeline):
    test_packets = [StringPacket('/chatter', ['Hi!', 'Hi!'])]
    simple_pipeline(test_packets)
    assert([] == test_packets[0].recieved)

def test_roscore__monitored_channel(simple_pipeline, launch_monitor):
    launch_monitor('monitor-single-parameter')
    test_packets = [StringPacket('/chatter', ['Hi!', 'Hi!'])]
    simple_pipeline(test_packets)
    assert(['Hi!RV1', 'Hi!RV2'] == test_packets[0].recieved)

def test_roscore__monitored_multiparam_channel(simple_pipeline, launch_monitor):
    launch_monitor('monitor-multiple-parameters')
    test_packets = [ColorRGBAPacket('/color_chatter', [str(ColorRGBA(100 ,100 ,100 ,0.5))])]
    simple_pipeline(test_packets)
    assert(['(105.0,110.0,115.0,0.5)'] == test_packets[0].recieved)

def test_roscore__monitored_multiparam_multichannel(simple_pipeline, launch_monitor):
    launch_monitor('monitor-multiple-channels')
    test_packets = [ ColorRGBAPacket('/color_chatter', [str(ColorRGBA(100 ,100 ,100 ,0.5))])
                   , StringPacket('/chatter', ['Hi!'])]
    simple_pipeline(test_packets)
    assert(['(105.0,110.0,115.0,0.5)'] == test_packets[0].recieved)
    assert(['Hi!RV']                   == test_packets[1].recieved)
