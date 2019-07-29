#include <iostream>
#include <string>
#include <boost/optional.hpp>
#include <boost/utility.hpp>
#include "rvmonitor.h"

class ROSTest
    : boost::noncopyable
{
public:
    ROSTest(char const * name)
        : test_name(name)
        , rate(30)
    {}

    ~ROSTest() {
        std::cout << test_name << " passed." << std::endl;
        wait();
    }

    void wait(uint n = 30) {
        for (uint i = 0; i < n; i ++) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    string const test_name;
public:
    ros::NodeHandle node;
    ros::Rate rate;
};

void test_unmonitored_channel()
{
    ROSTest t(__FUNCTION__);
    string const topic = "unmonitored";

    ros::Publisher pub = t.node.advertise<std_msgs::String>(topic, 1000);
    t.wait();

    boost::optional<std_msgs::String> msg_recvd;
    auto recieved_callback =
        [&msg_recvd] (std_msgs::String::ConstPtr const& msg) {
            msg_recvd = * msg;
        };
    ros::Subscriber sub = t.node.subscribe<std_msgs::String>(topic, 1000, recieved_callback);
    t.wait();

    string const msg_data = "Hello from publisher!\n";
    std_msgs::String msg_sent;
    msg_sent.data = msg_data;
    pub.publish(msg_sent);
    t.wait();

    if (!msg_recvd) { throw std::runtime_error("Response not recieved!"); }
    if (msg_recvd->data != msg_data) { throw std::runtime_error("Incorrect message data recieved"); }
}

void test_monitored_channel()
{
    ROSTest t(__FUNCTION__);
    string const topic = "/chatter";

    ros::Publisher pub = t.node.advertise<std_msgs::String>(topic, 1000);
    t.wait();

    boost::optional<std_msgs::String> msg_recvd;
    auto recieved_callback =
        [&msg_recvd] (std_msgs::String::ConstPtr const& msg) {
            msg_recvd = * msg;
        };
    ros::Subscriber sub = t.node.subscribe<std_msgs::String>(topic, 1000, recieved_callback);
    t.wait();

    string const msg_data = "Hello from publisher!\n";
    std_msgs::String msg_sent;
    msg_sent.data = msg_data;
    pub.publish(msg_sent);
    t.wait();

    if (!msg_recvd) { throw std::runtime_error("Response not recieved!"); }
    if (msg_recvd->data != msg_data + "RV") { throw std::runtime_error("Incorrect message data recieved: " + msg_recvd->data); }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    test_unmonitored_channel();
    test_monitored_channel();
    std::cout << "Passed.\n";
    return 0;
}
