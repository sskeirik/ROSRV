#include <iostream>
#include <string>
#include <boost/optional.hpp>

#include "rvmonitor.h"

void test_unmonitored_channel()
{
    int argc = 1; char* argv[] = { "test" };
    char const * node_name = __FUNCTION__;
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ros::Rate rate(10);

    string const topic = "unmonitored";

    ros::Publisher pub = n.advertise<std_msgs::String>(topic, 1000);
    ros::spinOnce();

    boost::optional<std_msgs::String> msg_recvd;
    auto recieved_callback =
        [&msg_recvd] (std_msgs::String::ConstPtr const& msg) {
            msg_recvd = * msg;
        };
    ros::Subscriber sub = n.subscribe<std_msgs::String>(topic, 1000, recieved_callback);
    ros::spinOnce();

    char const* msg_data = "Hello from publisher!\n";
    std_msgs::String msg_sent;
    msg_sent.data = msg_data;
    pub.publish(msg_sent);
    ros::spinOnce();

    rate.sleep();
    ros::spinOnce();

    if (!msg_recvd) { throw std::runtime_error("Response not recieved!"); }
    if (msg_recvd->data != msg_data) { throw std::runtime_error("Incorrect message data recieved"); }
}

int main()
{
    test_unmonitored_channel();
    return 0;
}

