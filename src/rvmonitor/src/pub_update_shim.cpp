#include "rv/pub_update_shim.h"
#include "ros/ros.h"
#include "ros/xmlrpc_manager.h"

#define private public
#define protected public
#include "ros/topic_manager.h"
#undef private
#undef protected

using namespace rv;
using namespace monitor;
using namespace ros;
using namespace XmlRpc;

PubUpdateShim::PubUpdateShim()
  : topic_manager()
{
  XMLRPCManagerPtr xmlrpc_manager = XMLRPCManager::instance();
//  xmlrpc_manager->bind("publisherUpdate", boost::bind(&TopicManager::pubUpdateCallback, this, _1, _2));

  TopicManagerPtr topic_manager = TopicManager::instance();
}

void PubUpdateShim::pubUpdateCallback(XmlRpcValue& params, XmlRpcValue& result)
{
  std::vector<std::string> pubs;
  for (int idx = 0; idx < params[2].size(); idx++)
  {
    pubs.push_back(params[2][idx]);
  }
  if (pubUpdate(params[1], pubs))
  {
    result = xmlrpc::responseInt(1, "", 0);
  }
  else
  {
    result = xmlrpc::responseInt(0, console::g_last_error_message, 0);
  }
}

bool pubUpdate(std::string topic, const std::vector<std::string> &pubs)
{
  if (isMonitored(topic)) {
    topic = getMonitorSubscribedTopicForTopic(topic);
    // ...
  }

  return topic_manager->pubUpdate(topic, pubs);
}
