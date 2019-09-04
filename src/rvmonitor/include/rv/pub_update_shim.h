#ifndef pub_update_shim_h_INCLUDED
#define pub_update_shim_h_INCLUDED

#include <ros/forwards.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace rv {
namespace monitor {

struct PubUpdateShim {
  PubUpdateShim();

  bool pubUpdate(std::string topic, const std::vector<std::string> &pubs);
  void pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

private:
  ros::TopicManagerPtr topic_manager;
};

}
}

#endif

