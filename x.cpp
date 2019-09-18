#include "rvmonitor.h"
#include <rv/monitor.h>
#include <cstring>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include <ros/console.h>

#include "std_msgs/String.h"
#include "std_msgs/String.h"
#include "std_msgs/String.h"
#include "std_msgs/String.h"

using namespace std;
namespace rosmop_generated
{
struct testSpecOne
{
  string const suffix = "RV";
  int messages_intercepted = 0;

  /* chatter_00_cerr */
  void callback_chatter_00_cerr(std::string& msg)
  {
    std::cerr << "Recieved message: " << msg << std::endl;
  }
  std::string& accessor_chatter_00_cerr(std_msgs::String& message)
  {
    return message.data;
  }
  /* chatter_0_append_suffix */
  void callback_chatter_0_append_suffix(std::string& msg)
  {
    msg = msg + suffix;
  }
  std::string& accessor_chatter_0_append_suffix(std_msgs::String& message)
  {
    return message.data;
  }
  /* chatter_1_append_num_intercepted */
  void callback_chatter_1_append_num_intercepted(std::string& msg)
  {
    messages_intercepted++;
    msg = msg + std::to_string(messages_intercepted);
  }
  std::string& accessor_chatter_1_append_num_intercepted(std_msgs::String& message)
  {
    return message.data;
  }
  /* chatter_2_ros_info */
  void callback_chatter_2_ros_info(std::string& msg)
  {
    ROS_INFO("%s", msg.c_str());
  }
  std::string& accessor_chatter_2_ros_info(std_msgs::String& message)
  {
    return message.data;
  }

  testSpecOne(rv::monitor::Monitor& monitor) {
    auto chatter = monitor.withTopic<std_msgs::String>("/chatter");
    chatter->registerEvent(this, &rosmop_generated::testSpecOne::callback_chatter_00_cerr,
                           &rosmop_generated::testSpecOne::accessor_chatter_00_cerr);
    chatter->registerEvent(this, &rosmop_generated::testSpecOne::callback_chatter_0_append_suffix,
                           &rosmop_generated::testSpecOne::accessor_chatter_0_append_suffix);
    chatter->registerEvent(this, &rosmop_generated::testSpecOne::callback_chatter_1_append_num_intercepted,
                           &rosmop_generated::testSpecOne::accessor_chatter_1_append_num_intercepted);
    chatter->registerEvent(this, &rosmop_generated::testSpecOne::callback_chatter_2_ros_info,
                           &rosmop_generated::testSpecOne::accessor_chatter_2_ros_info);
  }
};
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rvmonitor");
  rv::monitor::Monitor monitor;
  rosmop_generated::testSpecOne testSpecOne(monitor);
  monitor.enable_rvmaster_shims();
  ros::spin();
  return 0;
}
