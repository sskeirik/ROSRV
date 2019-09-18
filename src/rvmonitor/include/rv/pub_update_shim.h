#ifndef pub_update_shim_h_INCLUDED
#define pub_update_shim_h_INCLUDED

#include <ros/forwards.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace rv {
namespace monitor {

struct Monitor;

struct PubUpdateShim {
  PubUpdateShim(Monitor&);

  bool pubUpdate(std::string topic, std::vector<std::string> const&);
  void pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

private:
  ros::TopicManagerPtr topic_manager;
  Monitor& monitor;
};

}
}

#endif

