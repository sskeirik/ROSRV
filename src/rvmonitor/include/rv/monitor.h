#ifndef RV_MONITOR_H
#define RV_MONITOR_H

#include <string>
#include <functional>
#include <ros/ros.h> // TODO: Use more specific headers
#include <rv/subscription_shim.h>
#include <rv/pub_update_shim.h>
#include <boost/optional.hpp>
#include <ros/console.h>

namespace rv {
namespace monitor {

std::string getMonitorSubscribedTopicForTopic(const std::string& topic);
std::string getMonitorAdvertisedTopicForTopic(const std::string& topic);

struct ROSInit {
    ROSInit(int& argc, char** argv, std::string const& node_name)
    {
        ros::init(argc, argv, node_name);  
    }
};

struct MonitorTopicErased
{
    ros::Publisher  publisher;
    ros::Subscriber subscriber;
    rv::SubscriptionShim subscription_shim;

    MonitorTopicErased(std::string const& topic, ros::Publisher pub, ros::Subscriber sub)
        : publisher(pub)
        , subscriber(sub)
        , subscription_shim(topic, getMonitorSubscribedTopicForTopic(topic))
    {
    }

    virtual ~MonitorTopicErased() = default;
};

using MonitorTopicErasedPtr = boost::shared_ptr<MonitorTopicErased>;

template<class MessageType>
struct MonitorTopic
    : MonitorTopicErased
{
    using Ptr = boost::shared_ptr<MonitorTopic<MessageType>>;

    MonitorTopic(ros::NodeHandle& n, std::string const& topic, uint queue_len)
        : MonitorTopicErased
            ( topic
            , n.advertise<MessageType>(getMonitorAdvertisedTopicForTopic(topic), queue_len, true)
            , n.subscribe( getMonitorSubscribedTopicForTopic(topic)
                                 , queue_len
                                 , &MonitorTopic<MessageType>::callback
                                 , this
            )            )
    {
    }

    template<class T>
    void registerEvent( T* owner
                      , void (T::*callback)(MessageType&)
                      )
    {
        auto cb = [owner, callback](MessageType& msg) -> void { (owner->*callback)(msg); };
        m_events.push_back(cb);
    }

    void callback(boost::shared_ptr<const MessageType> ptr) {
        MessageType copy = *ptr;
        for (auto event_cb: m_events) { event_cb(copy); }
        publisher.publish(copy);
    }

private:
    std::vector<std::function<void (MessageType&)>> m_events;
};

struct Monitor {
    Monitor(int argc, char** argv, std::string const& node_name)
        : ros_init(argc, argv, "rvmonitor")
    {
        std::cerr << "Monitor constructed " << "\n";
        std::cerr << "argv: " << argv[0] << '\n';


        if (argc >= 2 && std::string(argv[1]) == "--with-rvmaster")
            enable_rvmaster_shims();
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
               ros::console::notifyLoggerLevelsChanged();
        }
    }

    /* Create a MonitorTopic, make sure that it is of the right message type */
    template<class MessageType>
    typename MonitorTopic<MessageType>::Ptr withTopic(std::string const& topic) {
        typename MonitorTopic<MessageType>::Ptr ret = nullptr;
        if (monitored_topics.find(topic) == monitored_topics.end()) {
            unsigned int const queue_len = 1000;
            ret = boost::make_shared<MonitorTopic<MessageType>>(node_handle, topic, queue_len);
            monitored_topics.insert({topic, ret});
        }
        else {
            ret = boost::dynamic_pointer_cast<MonitorTopic<MessageType>>(monitored_topics.at(topic));
        }
        return ret;
    }

    /* Register a handler for an topic */
    template<class MessageType, class T>
    void registerEvent(std::string const& topic, T* owner, void (T::*callback)(MessageType&)) {
        auto monitor_topic = withTopic<MessageType>(topic); 
        monitor_topic->registerEvent(owner, callback);
    }

    void enable_rvmaster_shims()
    {
        std::cerr << "Shim enabled\n";
        pub_update_shim.emplace(*this); // Construct a new PubUpdateShim
    };

    bool isMonitored(std::string const& topic) {
        return monitored_topics.find(topic) != monitored_topics.end();
    }

    int run() {
        ros::spin();
        return 0;
    }

    ROSInit ros_init;
    ros::NodeHandle node_handle;
    boost::optional<PubUpdateShim> pub_update_shim;
    std::map<std::string, MonitorTopicErasedPtr> monitored_topics;
};

}
}

#endif

