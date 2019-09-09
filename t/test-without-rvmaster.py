#!/usr/bin/env python2

import rospy as ros
from subprocess import Popen, check_call
from std_msgs.msg import String, ColorRGBA

import pytest

@pytest.fixture(scope="session")
def ros_init():
    ros.init_node('test')

@pytest.fixture()
def ros_subscribe(ros_init):
    def subscribe(topic, msg_type, to_formatted_string):
        class context: # see https://stackoverflow.com/a/28433571/1278288
            recieved = []
        def callback(msg):
            recieved_str = to_formatted_string(msg)
            ros.loginfo("Recieved: %s", recieved_str)
            context.recieved += [recieved_str]
        rate = ros.Rate(10) # Hz
        ros.Subscriber(topic, msg_type, callback)
        rate.sleep()
        return [ros_init, rate, context.recieved]
    return subscribe

@pytest.fixture()
def launch_monitor(request):
    def launch(name):
        monitor = Popen(['/usr/bin/env', 'rosrun', 'rvmonitor', name])
        def cleanup():
            monitor.terminate()
        request.addfinalizer(cleanup)
        return monitor
    return launch

## Tests without RVMaster

def to_formatted_string__String(msg):
    return msg.data

def test_roscore__unmonitored_channel(ros_subscribe):
    [ros_init, rate, recieved_messages] = ros_subscribe('unmonitored', String, to_formatted_string__String)
    check_call(['rostopic', 'pub', '--once', '/unmonitored', 'std_msgs/String', 'Hi!'])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == ['Hi!'])

def test_roscore__monitored_channel__no_monitor_running(ros_subscribe):
    [ros_init, rate, recieved_messages] = ros_subscribe('monitored', String, to_formatted_string__String)
    check_call(['rostopic', 'pub', '--once', '/chatter', 'std_msgs/String', 'Hi!'])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == [])

def test_roscore__monitored_channel__monitor_running(ros_subscribe, launch_monitor):
    [ros_init, rate, recieved_messages] = ros_subscribe('chatter', String, to_formatted_string__String)
    launch_monitor('monitor-single-parameter')
    check_call(['rostopic', 'pub', '--once', '/rv/monitored/chatter', 'std_msgs/String', 'Hi!'])
    check_call(['rostopic', 'pub', '--once', '/rv/monitored/chatter', 'std_msgs/String', 'Hi!'])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == ['Hi!RV1', 'Hi!RV2'])

def to_formatted_string__colorRGBA(msg):
    return '({},{},{},{})'.format(msg.r, msg.g, msg.b, msg.a)

def test_roscore__monitored_multiparam_channel__no_monitor_running(ros_subscribe, launch_monitor):
    [ros_init, rate, recieved_messages] = ros_subscribe('color_chatter', ColorRGBA, to_formatted_string__colorRGBA)
    launch_monitor('monitor-multiple-parameters')
    test_msg = ColorRGBA(100, 100, 100, 0.5)
    check_call(['rostopic', 'pub', '--once', '/color_chatter', 'std_msgs/ColorRGBA', str(test_msg)])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == ['(100.0,100.0,100.0,0.5)'])

def test_roscore__monitored_multiparam_channel__monitor_running(ros_subscribe, launch_monitor):
    [ros_init, rate, recieved_messages] = ros_subscribe('color_chatter', ColorRGBA, to_formatted_string__colorRGBA)
    launch_monitor('monitor-multiple-parameters')
    test_msg = ColorRGBA(100, 100, 100, 0.5)
    check_call(['rostopic', 'pub', '--once', '/rv/monitored/color_chatter', 'std_msgs/ColorRGBA', str(test_msg)])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == ['(105.0,110.0,115.0,0.5)'])




