#include <thread>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "rosinterface_handler/smart_subscriber.hpp"

using Msg = std_msgs::Int32;
class TestNode {
public:
    TestNode() {
        ros::NodeHandle nh;
        pub = nh.advertise<Msg>("/output", 5);
        sub = nh.subscribe("/input", 5, &TestNode::msgCallback, this);
        smartSub.subscribe(nh, "/input", 5);
        smartSub.addPublisher(pub);
        smartSub.registerCallback(boost::bind(&TestNode::smartMsgCallback, this, _1));
    }
    int msgCount{0};
    int smartMsgCount{0};
    ros::Publisher pub;
    ros::Subscriber sub;
    rosinterface_handler::SmartSubscriber<std_msgs::Int32> smartSub;

private:
    void msgCallback(const Msg::ConstPtr& msg) {
        msgCount++;
        pub.publish(msg);
    }
    void smartMsgCallback(const Msg::ConstPtr& /*msg*/) {
        smartMsgCount++;
    }
};
void intCb(const std_msgs::Int32& /*msg*/) {
}

class ImageNode {
    using SmartSub = rosinterface_handler::SmartSubscriber<sensor_msgs::Image>;

public:
    ImageNode() : transport(ros::NodeHandle()) {
        image_transport::SubscriberStatusCallback cb = boost::bind(&SmartSub::subscribeCallback, &smartSub);
        pub = transport.advertise("/img_out", 5, cb, cb);
        ros::NodeHandle nh;
        smartSub.subscribe(nh, "/img_in", 5);
        smartSub.addPublisher(pub);
        smartSub.registerCallback(boost::bind(&ImageNode::imgCallback, this, _1));
    }
    int msgCount{0};
    int smartMsgCount{0};
    image_transport::ImageTransport transport;
    image_transport::Publisher pub;
    SmartSub smartSub;

private:
    void imgCallback(const sensor_msgs::Image::ConstPtr& msg) {
        msgCount++;
        pub.publish(msg);
    }
};
void cb(const sensor_msgs::Image& /*msg*/) {
}

template <typename Func>
bool tryRepeatedly(Func&& f) {
    int i = 0;
    while (!f() && i++ < 20) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return i <= 20;
}

TEST(SmartSubscriber, subscribeTests) {
    // subscribe to a topic twice (smart and non smart)
    // make sure the smart subscriber publishes no messages after rostopic echo stops listening
    TestNode node;
    ros::Subscriber listener = ros::NodeHandle().subscribe("/output", 5, intCb);
    // check we are subscribed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    node.msgCount = 0;
    node.smartMsgCount = 0;
    EXPECT_LT(0, node.pub.getNumSubscribers());
    EXPECT_TRUE(node.smartSub.isSubscribed());

    // test that we similar numbers of msgs than with a normal subscriber
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_NEAR(node.msgCount, node.smartMsgCount, 5);
    EXPECT_TRUE(node.smartSub.isSubscribed());
    EXPECT_GT(node.smartMsgCount, 0);
    listener.shutdown();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // by now surely no one will listen
    const int oldSmartMsgCount = node.smartMsgCount;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_FALSE(node.smartSub.isSubscribed());
    EXPECT_NEAR(oldSmartMsgCount, node.smartMsgCount, 2);
}

TEST(SmartSubscriber, basicTests) {
    ros::NodeHandle nh;
    ros::Publisher myPub = nh.advertise<Msg>("/output", 5);
    ros::Publisher myPub2 = nh.advertise<Msg>("/output2", 5);
    rosinterface_handler::SmartSubscriber<Msg> sub(myPub);
    sub.subscribe(nh, "/input", 5);
    EXPECT_TRUE(sub.removePublisher(myPub.getTopic()));
    EXPECT_FALSE(sub.removePublisher(myPub2.getTopic()));
    sub.addPublisher(myPub2);
    EXPECT_TRUE(sub.removePublisher(myPub2.getTopic()));
}

TEST(SmartSubscriber, nondefaultPublisher) {
    ImageNode node;
    EXPECT_FALSE(node.smartSub.isSubscribed());
    ros::NodeHandle nh;
    // check we subscribe
    auto testIsSubscribed = [&]() {
        ros::spinOnce();
        return node.smartSub.isSubscribed();
    };
    {
        auto sub = nh.subscribe("/img_out", 5, cb);
        // usually this works instantly but on high-load systems it may take longer
        EXPECT_TRUE(tryRepeatedly(testIsSubscribed));
    }
    // check we are no longer subscribed
    auto testNotSubscribed = [&]() { return !testIsSubscribed(); };
    EXPECT_TRUE(tryRepeatedly(testNotSubscribed));

    // force subscription
    node.smartSub.setSmart(false);
    EXPECT_FALSE(node.smartSub.smart());
    EXPECT_TRUE(node.smartSub.isSubscribed());
}
