#include <iostream>

#include "ros/file_log.h"
#include "ros/network.h"
#include "ros/ros.h"
#include <cstdint>
#include "rv/subscription_shim.h"

/* Allow usage of private members of internal ros APIs
 * WARNING: This is undefined behaviour.
 */
#define private public
#define protected public
#include "ros/connection.h"
#include "ros/connection_manager.h"
#include "ros/poll_manager.h"
#include "ros/publisher_link.h"
#include "ros/subscription.h"
#include "ros/topic_manager.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport_publisher_link.h"
#undef private
#undef protected

using namespace std;
using namespace ros;
using namespace rv;
using namespace XmlRpc;

SubscriptionShim::SubscriptionShim(std::string const& connectTopic, std::string const& handlerTopic)
  : connectTopic(connectTopic)
  , handlerTopic(handlerTopic)
  , topic_manager(ros::TopicManager::instance())
{
}

SubscriptionPtr SubscriptionShim::getSubscriptionForTopic(string const& topic)
{
  if (topic_manager->isShuttingDown())
  {
    return nullptr;
  }

  for (L_Subscription::iterator s = topic_manager->subscriptions_.begin(); s != topic_manager->subscriptions_.end(); ++s)
  {
    SubscriptionPtr candidate = *s;
    if (!candidate->isDropped() && candidate->getName() == topic)
    {
      return candidate;
    }
  }
  return nullptr;
}

bool SubscriptionShim::executeRequestTopic(SubscriptionPtr subscription, std::string const& xmlrpc_uri,
                                           XmlRpc::XmlRpcValue& proto)
{
  string peer_host;
  uint32_t peer_port;
  if (!network::splitURI(xmlrpc_uri, peer_host, peer_port))
  {
    ROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
    return false;
  }

  XmlRpcValue tcpros_array, protos_array, params;
  tcpros_array[0] = std::string("TCPROS");
  protos_array[0] = tcpros_array;
  params[0] = this_node::getName();
  params[1] = connectTopic;
  params[2] = protos_array;

  TransportUDPPtr udp_transport = nullptr;
  XmlRpcClient* c = new XmlRpcClient(peer_host.c_str(), peer_port, "/");
  // Initiate the negotiation
  XmlRpcValue requestTopicResult;
  if (!c->execute("requestTopic", params, requestTopicResult))
  {
    ROSCPP_LOG_DEBUG("Failed to contact publisher [%s:%d] for topic [%s]", peer_host.c_str(), peer_port,
                     connectTopic.c_str());
    delete c;
    assert(!udp_transport);
    return false;
  }

  boost::mutex::scoped_lock lock(subscription->shutdown_mutex_);
  if (subscription->shutting_down_ || subscription->dropped_)
  {
    return false;
  }

  if (!XMLRPCManager::instance()->validateXmlrpcResponse("requestTopic", requestTopicResult, proto))
  {
    ROSCPP_LOG_DEBUG("Failed to contact publisher [%s:%d] for topic [%s]", peer_host.c_str(), peer_port,
                     connectTopic.c_str());
    assert(!udp_transport);
    return false;
  }

  return true;
}

bool SubscriptionShim::startROSTCPConnection(SubscriptionPtr subscription, std::string const& xmlrpc_uri,
                                             XmlRpc::XmlRpcValue proto)
{
  TransportUDPPtr udp_transport = nullptr;
  if (proto.size() == 0)
  {
    ROSCPP_LOG_DEBUG("Couldn't agree on any common protocols with [%s] for topic [%s]", xmlrpc_uri.c_str(),
                     connectTopic.c_str());
    assert(!udp_transport);
    return false;
  }

  if (proto.getType() != XmlRpcValue::TypeArray)
  {
    ROSCPP_LOG_DEBUG("Available protocol info returned from %s is not a list.", xmlrpc_uri.c_str());
    assert(!udp_transport);
    return false;
  }

  if (proto[0].getType() != XmlRpcValue::TypeString)
  {
    ROSCPP_LOG_DEBUG("Available protocol info list doesn't have a string as its first element.");
    assert(!udp_transport);
    return false;
  }

  std::string proto_name = proto[0];
  if (proto_name != "TCPROS")
  {
    ROSCPP_LOG_DEBUG("RV Monitors only support TCPROS when RVMaster is running");
    assert(!udp_transport);
    return false;
  }

  if (proto.size() != 3 || proto[1].getType() != XmlRpcValue::TypeString || proto[2].getType() != XmlRpcValue::TypeInt)
  {
    ROSCPP_LOG_DEBUG("publisher implements TCPROS, but the "
                     "parameters aren't string,int");
    return false;
  }
  std::string pub_host = proto[1];
  int pub_port = proto[2];
  ROSCPP_CONN_LOG_DEBUG("Connecting via tcpros to topic [%s] at host [%s:%d]", connectTopic.c_str(), pub_host.c_str(),
                        pub_port);

  TransportTCPPtr transport(boost::make_shared<TransportTCP>(&PollManager::instance()->getPollSet()));
  if (!transport->connect(pub_host, pub_port))
  {
    ROSCPP_CONN_LOG_DEBUG("Failed to connect to publisher of topic [%s] at [%s:%d]", connectTopic.c_str(),
                          pub_host.c_str(), pub_port);
  }

  ConnectionPtr connection(boost::make_shared<Connection>());
  TransportPublisherLinkPtr pub_link(
      boost::make_shared<TransportPublisherLink>(subscription, xmlrpc_uri, subscription->transport_hints_));

  connection->initialize(transport, false, HeaderReceivedFunc());

  pub_link->connection_ = connection;
  // slot_type is used to automatically track the TransporPublisherLink class' existence
  // and disconnect when this class' reference count is decremented to 0. It increments
  // then decrements the shared_from_this reference count around calls to the
  // onConnectionDropped function, preventing a coredump in the middle of execution.
  connection->addDropListener(
      Connection::DropSignal::slot_type(&TransportPublisherLink::onConnectionDropped, pub_link.get(), _1, _2)
          .track(pub_link));

  if (connection->getTransport()->requiresHeader())
  {
    connection->setHeaderReceivedCallback(
        boost::bind(&TransportPublisherLink::onHeaderReceived, pub_link.get(), _1, _2));

    M_string header;
    std::cerr << "Writing " << connectTopic << " to header\n";
    header["topic"] = connectTopic;
    header["md5sum"] = subscription->md5sum();
    header["callerid"] = this_node::getName();
    header["type"] = subscription->datatype();
    header["tcp_nodelay"] = pub_link->transport_hints_.getTCPNoDelay() ? "1" : "0";
    connection->writeHeader(header, boost::bind(&TransportPublisherLink::onHeaderWritten, pub_link.get(), _1));
  }
  else
  {
    connection->read(4, boost::bind(&TransportPublisherLink::onMessageLength, pub_link.get(), _1, _2, _3, _4));
  }

  ConnectionManager::instance()->addConnection(connection);

  {
    boost::mutex::scoped_lock lock(subscription->publisher_links_mutex_);
    subscription->addPublisherLink(pub_link);
  }

  ROSCPP_CONN_LOG_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", connectTopic.c_str(), pub_host.c_str(),
                        pub_port);

  return true;
}

/*
 * Negotiate a connection for a topic and place it in the subscription for another.
 */
bool SubscriptionShim::connect(std::string const& xmlrpc_uri)
{
  std::cerr << "Connecting to: " << xmlrpc_uri << std::endl;
  XmlRpcValue proto;
  SubscriptionPtr subscription = getSubscriptionForTopic(handlerTopic);
  if (!executeRequestTopic(subscription, xmlrpc_uri, proto)) return false;
  if (!startROSTCPConnection(subscription, xmlrpc_uri, proto)) return false;
  return true;
}

bool SubscriptionShim::connectToPublishers()
{
  SubscriptionPtr subscription = getSubscriptionForTopic(handlerTopic);

  std::vector<string> uris;
  {
    boost::mutex::scoped_lock lock(subscription->publisher_links_mutex_);
    for (auto spc = subscription->publisher_links_.begin();
         spc != subscription->publisher_links_.end(); ++spc)
    {
      uris.push_back((*spc)->getPublisherXMLRPCURI());
    }
  }

  for (string const& uri: uris) {
    connect(uri);
  }

  {
    boost::mutex::scoped_lock lock(subscription->pending_connections_mutex_);
    auto it = subscription->pending_connections_.begin();
    auto end = subscription->pending_connections_.end();
    for (; it != end; ++it)
    {
      connect((*it)->getRemoteURI());
    }
  }
  return true;
}
