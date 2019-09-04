#include "rv/pub_update_shim.h"
#include "ros/xmlrpc_manager.h"

#define private public
#define protected public
#include "ros/topic_manager.h"
#undefine private
#undefine protected


using namespace rv;
using namespace monitor;

PubUpdateShim::PubUpdateShim()
  : topic_manager()
{
  XMLRPCManagerPtr xmlrpc_manager = XMLRPCManager::instance();
  xmlrpc_manager->bind("publisherUpdate", boost::bind(&TopicManager::pubUpdateCallback, this, _1, _2));

  XMLRPCManagerPtr topic_manager = TopicManager::instance();
}

void PubUpdateShim::pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
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
  }

  return topic_manager->pubUpdate(topic, pubs); }
}
