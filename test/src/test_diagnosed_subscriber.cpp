#include <geometry_msgs/PointStamped.h>
#include <gtest/gtest.h>
#include <rosinterface_handler/diagnostic_subscriber.hpp>
#include <rosinterface_handler/smart_subscriber.hpp>

using MsgT = geometry_msgs::PointStamped;

class DummyUpdater : public diagnostic_updater::Updater {
public:
    void forceUpdate() {
        statusVec.clear();
        for (const auto& task : getTasks()) {
            diagnostic_updater::DiagnosticStatusWrapper status;

            status.name = task.getName();
            status.level = 2;
            status.message = "No message was set";
            status.hardware_id = "none";

            task.run(status);
            statusVec.push_back(status);
        }
    }
    std::vector<diagnostic_msgs::DiagnosticStatus> statusVec;
};

using DiagPub = rosinterface_handler::DiagnosedPublisher<MsgT>;
using DiagSub = rosinterface_handler::DiagnosedSubscriber<MsgT>;
class TestDiagnosedPubSub : public testing::Test {
protected:
    void SetUp() override {
        updater.statusVec.clear();
        pub = nh.advertise<MsgT>("test_topic", 5);
        sub.subscribe(nh, "test_topic", 5);
        pub.minFrequency(10).maxTimeDelay(1);
        sub.minFrequency(10).maxTimeDelay(1);
        messageCounter = 0;
    }

public:
    ros::NodeHandle nh;
    DummyUpdater updater;
    DiagPub pub{updater};
    DiagSub sub{updater};
    int messageCounter{0};
};

TEST_F(TestDiagnosedPubSub, publishAndReceiveOK) {
    auto onTimer = [this](const ros::TimerEvent& e) {
        this->messageCounter++;
        auto msg = boost::make_shared<MsgT>();
        msg->header.stamp = e.current_real;
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.05), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.forceUpdate();
    ASSERT_GE(4, updater.statusVec.size());
    for (auto& status : updater.statusVec) {
        EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, status.level);
    }
}

TEST_F(TestDiagnosedPubSub, publishAndReceiveFail) {
    auto onTimer = [this](const ros::TimerEvent& e) {
        this->messageCounter++;
        auto msg = boost::make_shared<MsgT>();
        msg->header.stamp = e.current_real - ros::Duration(1.5);
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.15), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.forceUpdate();
    ASSERT_LE(2, updater.statusVec.size());
    for (auto& status : updater.statusVec) {
        EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, status.level);
    }
}

TEST_F(TestDiagnosedPubSub, overrideTopic) {
    this->sub.subscribe(nh, "new_topic", 5);
    this->pub = nh.advertise<MsgT>("new_topic", 5);
    auto onTimer = [this](const ros::TimerEvent& e) {
        this->messageCounter++;
        auto msg = boost::make_shared<MsgT>();
        msg->header.stamp = e.current_real;
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.05), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.forceUpdate();
    ASSERT_LE(2, updater.statusVec.size());
    for (auto& status : updater.statusVec) {
        EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, status.level);
    }
}

TEST_F(TestDiagnosedPubSub, assign) {
    this->pub = DiagPub(updater);
    auto onTimer = [this](const ros::TimerEvent& e) {
        this->messageCounter++;
        auto msg = boost::make_shared<MsgT>();
        msg->header.stamp = e.current_real;
        this->pub.publish(msg);
    };
    auto timer = nh.createTimer(ros::Duration(0.05), onTimer);
    while (messageCounter < 10) {
        ros::spinOnce();
    }
    updater.forceUpdate();
    ASSERT_LE(1, updater.statusVec.size());
}

TEST(TestDiagnosedAndSmartCombined, constructAndSubscribe) {
    ros::NodeHandle nh;
    ros::Publisher thisPublisher = nh.advertise<MsgT>("sometopic", 5);
    using Subscriber = rosinterface_handler::DiagnosedSubscriber<MsgT, rosinterface_handler::SmartSubscriber<MsgT>>;
    DummyUpdater updater;
    Subscriber s(updater, thisPublisher);
    EXPECT_FALSE(s.isSubscribed());
    s.subscribe(nh, "atopic", 5);
    EXPECT_FALSE(s.isSubscribed());
    ros::Subscriber otherSubscriber = nh.subscribe("sometopic", 5, +[](const MsgT::ConstPtr& msg) {});
    ros::spinOnce();
    ASSERT_GT(thisPublisher.getNumSubscribers(), 0);
    s.subscribeCallback();
    EXPECT_TRUE(s.isSubscribed());
}
