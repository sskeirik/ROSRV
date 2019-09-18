#include "rv/pub_update_shim.h"
#include "ros/ros.h"
#include "ros/xmlrpc_manager.h"
#include "rv/monitor.h"

#define private public
#define protected public
#include "ros/topic_manager.h"
#undef private
#undef protected

using namespace rv;
using namespace monitor;
using namespace ros;
using namespace XmlRpc;
using namespace std;

PubUpdateShim::PubUpdateShim(Monitor& m)
  : topic_manager()
  , monitor(m)
{
  XMLRPCManagerPtr xmlrpc_manager = XMLRPCManager::instance();
  xmlrpc_manager->bind("publisherUpdate", boost::bind(&PubUpdateShim::pubUpdateCallback, this, _1, _2));

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

bool PubUpdateShim::pubUpdate(std::string topic, const std::vector<std::string> &uris)
{
  std::cerr << "_-------------------" << __FUNCTION__ << "-----------------\n";
  if (monitor.isMonitored(topic)) {
    topic = getMonitorSubscribedTopicForTopic(topic);
    for (auto uri: uris) {
        monitor.monitored_topics.at(topic)->subscription_shim.connect(uri);
    }
    return true;
  }

  return topic_manager->pubUpdate(topic, uris);
}
