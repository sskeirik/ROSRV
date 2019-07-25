#include <iostream>
#include <string>
#include <boost/optional.hpp>
#include <boost/utility.hpp>
#include "rvmonitor.h"

class ROSInit
    : boost::noncopyable
{
public:
    ROSInit(char const* name)
    {
        std::map<string, string> const remapping_args;
        uint32_t const options = 0;
        ros::init(remapping_args, name, options);
    }
};

class ROSTest
    : boost::noncopyable
{
public:
    ROSTest(char const * name)
        : test_name(name)
        , ros_init(name)
        , rate(10)
    {}

    ~ROSTest() {
        std::cout << test_name << " passed." << std::endl;
    }

private:
    string const test_name;
    ROSInit ros_init;
public:
    ros::NodeHandle node;
    ros::Rate rate;
};

void test_unmonitored_channel()
{
    ROSTest t(__FUNCTION__);
    string const topic = "unmonitored";

    ros::Publisher pub = t.node.advertise<std_msgs::String>(topic, 1000);
    ros::spinOnce();

    boost::optional<std_msgs::String> msg_recvd;
    auto recieved_callback =
        [&msg_recvd] (std_msgs::String::ConstPtr const& msg) {
            msg_recvd = * msg;
        };
    ros::Subscriber sub = t.node.subscribe<std_msgs::String>(topic, 1000, recieved_callback);
    ros::spinOnce();

    char const* msg_data = "Hello from publisher!\n";
    std_msgs::String msg_sent;
    msg_sent.data = msg_data;
    pub.publish(msg_sent);
    ros::spinOnce();

    t.rate.sleep();
    ros::spinOnce();

    if (!msg_recvd) { throw std::runtime_error("Response not recieved!"); }
    if (msg_recvd->data != msg_data) { throw std::runtime_error("Incorrect message data recieved"); }
}

int main()
{
    test_unmonitored_channel();
    std::cout << "Passed.\n";
    return 0;
}
