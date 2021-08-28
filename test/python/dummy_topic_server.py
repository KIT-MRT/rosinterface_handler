#!/usr/bin/env python

import rospy
from rosinterface_handler.srv import GetTopic, GetTopicRequest

class TopicServer:
    def __init__(self):
        self.requsted_topics = {}

    def check_topic_type(self, topic, hash):
        """Make sure message hash for newly requested topics matches the ones from older requests"""
        if topic in self.requsted_topics:
            assert self.requsted_topics[topic] == hash
        self.requsted_topics[topic] = hash


    def get_topic(self, request):
        """
        This is a very simple implementation of a topic server where we just log the request, check that requests
        for the same topic match and return the topic already proposed by the client.

        Usually this would be a much more sophisticated implementation to figure out the topic and check for misconfiguration,
        but the purpose of the test is to test the handling on the interface side, not the server.

        :param request: ros service request
        :type request: GetTopicRequest
        :return: the topic
        :rtype: str
        """
        pub_sub = "subscribe to" if request.pub_sub == request.subscriber else "publish"
        rospy.loginfo(
            "Topic server request from {} to {} for topic {}, type {}".format(
                request.node_name, pub_sub, request.topic_name, request.message_type
            )
        )
        self.check_topic_type(request.proposed_topic, request.md5sum)
        return request.proposed_topic


if __name__ == "__main__":
    rospy.init_node("topic_server")
    server = TopicServer()
    s = rospy.Service("~get_topic", GetTopic, server.get_topic)
    rospy.loginfo("Topic server: Service now available at {}".format(s.resolved_name))
    rospy.spin()
