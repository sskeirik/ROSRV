#include "rv/monitor.h"

using namespace std;
using namespace rv;

string monitor::getMonitorSubscribedTopicForTopic(const std::string& topic) {
    return "/rv/monitored" + topic;
}

string monitor::getMonitorAdvertisedTopicForTopic(const std::string& topic) {
    return topic;
}

