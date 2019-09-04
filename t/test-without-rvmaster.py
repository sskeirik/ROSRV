#!/usr/bin/env python2

import rospy as ros
from subprocess import check_call
from std_msgs.msg import String

import pytest

@pytest.fixture(scope="session")
def ros_init():
    ros.init_node('test')

@pytest.fixture()
def ros_subscribe(ros_init):
    def subscribe(topic):
        class context: # see https://stackoverflow.com/a/28433571/1278288
            recieved = []
        def callback(msg):
            ros.loginfo("Recieved: %s", msg.data)
            context.recieved += [msg.data]
        rate = ros.Rate(10) # Hz
        ros.Subscriber(topic, String, callback)
        rate.sleep()
        return [ros_init, rate, context.recieved]
    return subscribe

## Tests without RVMaster

def test_roscore__unmonitored_channel(ros_subscribe):
    [ros_init, rate, recieved_messages] = ros_subscribe('unmonitored') 
    check_call(['rostopic', 'pub', '--once', '/unmonitored', 'std_msgs/String', 'Hi!'])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == ['Hi!'])

# TODO: Test framework does not allow running without monitor
# def test_roscore__monitored_channel__no_monitor_running(ros_subscribe):
#     [ros_init, rate, recieved_messages] = ros_subscribe('monitored') 
#     check_call(['rostopic', 'pub', '--once', '/chatter', 'std_msgs/String', 'Hi!'])
#     rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
#     assert(recieved_messages == [])

def test_roscore__monitored_channel__monitor_running(ros_subscribe):
    [ros_init, rate, recieved_messages] = ros_subscribe('/chatter') 
    check_call(['rostopic', 'pub', '--once', '/rv/monitored/chatter', 'std_msgs/String', 'Hi!'])
    check_call(['rostopic', 'pub', '--once', '/rv/monitored/chatter', 'std_msgs/String', 'Hi!'])
    rate.sleep(); rate.sleep(); rate.sleep(); rate.sleep()
    assert(recieved_messages == ['Hi!RV1', 'Hi!RV2'])
