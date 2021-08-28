#pragma once
#include <utility>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <rosinterface_handler/GetTopic.h>
#include <rosinterface_handler/utilities.hpp>

namespace rosinterface_handler {

class TopicService {
public:
    TopicService(ros::NodeHandle& privNodeHandle) : nodeName_{privNodeHandle.getNamespace()} {
        std::string topicServer;
        ros::param::get("/topic_server", topicServer);
        if (!topicServer.empty()) {
            topicService_ = privNodeHandle.serviceClient<GetTopic>(topicServer + "/get_topic");
        }
    }

    template <typename Logger>
    void waitAvailable(const Logger& logger) {
        if (topicService_.isValid()) {
            logger.logDebug("Waiting for topic server service %s to be available...", topicService_.getService());
            topicService_.waitForExistence();
            logger.logDebug("Topic server service now available");
        }
    }

    template <typename TopicType>
    std::string getTopic(const char* topicName, const std::string& defaultNs, const std::string& defaultTopic) {
        auto topic = rosinterface_handler::getTopic(defaultNs, defaultTopic);
        if (topicService_.isValid()) {
            GetTopicResponse response;
            auto request = prepareRequest<TopicType>(topicName, std::move(topic));
            if (!topicService_.call(request, response)) {
                rosinterface_handler::exit("Failed to communicate with topic server!");
            }
            return std::move(response.topic); // no copy elision on subobjects
        }
        return topic;
    }

private:
    template <typename TopicType>
    GetTopicRequest prepareRequest(const char* topicName, std::string defaultTopic) {
        GetTopicRequest request;
        request.node_name = nodeName_;
        request.topic_name = topicName;
        request.proposed_topic = std::move(defaultTopic);
        request.md5sum = ros::message_traits::md5sum<TopicType>();
        request.message_type = ros::message_traits::datatype<TopicType>();
        return request;
    }
    ros::ServiceClient topicService_;
    std::string nodeName_;
};

} // namespace rosinterface_handler
