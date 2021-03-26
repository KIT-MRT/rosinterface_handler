#include <gtest/gtest.h>
#include <rosinterface_handler/DefaultsInterface.h>
#include <rosinterface_handler/simple_node_status.hpp>

using IfType = rosinterface_handler::DefaultsInterface;
using ConfigType = rosinterface_handler::DefaultsConfig;

TEST(RosinterfaceHandler, DefaultParams) { // NOLINT(readability-function-size)
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    ASSERT_EQ("info", testInterface.verbosity_param_w_default);

    ASSERT_EQ(1, testInterface.int_param_w_default);
    ASSERT_DOUBLE_EQ(1.1, testInterface.double_param_w_default);
    ASSERT_EQ("Hello World", testInterface.str_param_w_default);
    ASSERT_EQ(true, testInterface.bool_param_w_default);
    ASSERT_EQ(1L, testInterface.long_param_w_default_int);
    ASSERT_EQ(1L, testInterface.long_param_w_default_int_str);
    ASSERT_EQ(2147483648L, testInterface.long_param_w_default_long_string);

    ASSERT_EQ(std::vector<int>({1, 2, 3}), testInterface.vector_int_param_w_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testInterface.vector_double_param_w_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testInterface.vector_bool_param_w_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testInterface.vector_string_param_w_default);

    std::map<std::string, std::string> tmp{{"Hello", "World"}};
    ASSERT_EQ(tmp, testInterface.map_param_w_default);

    ASSERT_EQ(1, testInterface.enum_int_param_w_default);
    ASSERT_EQ("One", testInterface.enum_str_param_w_default);
    testInterface.showNodeInfo();
}

TEST(RosinterfaceHandler, NodeStatus) {
    IfType testInterface(ros::NodeHandle("~"));
    testInterface.nodeStatus.set(rosinterface_handler::NodeStatus::ERROR, "Test error");
    testInterface.nodeStatus.set(rosinterface_handler::NodeStatus::OK, "Initialized");
}

TEST(RosinterfaceHandler, DefaultSubscriber) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    ASSERT_TRUE(!!testInterface.subscriber_w_default);
    ASSERT_EQ(testInterface.subscriber_w_default->getTopic(), "/test/rosinterface_handler_test/in_topic");

    ASSERT_TRUE(!!testInterface.subscriber_public_w_default);
    ASSERT_EQ(testInterface.subscriber_public_w_default->getTopic(), "/test/in_topic");

    ASSERT_TRUE(!!testInterface.subscriber_global_w_default);
    ASSERT_EQ(testInterface.subscriber_global_w_default->getTopic(), "/in_topic");
}

TEST(RosinterfaceHandler, DefaultPublisher) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    ASSERT_EQ(testInterface.publisher_w_default.getTopic(), "/test/rosinterface_handler_test/out_topic");
    ASSERT_EQ(testInterface.publisher_public_w_default.getTopic(), "/test/out_topic");
    ASSERT_EQ(testInterface.publisher_global_w_default.getTopic(), "/out_topic");
}

TEST(RosinterfaceHandler, DefaultsOnParamServer) { // NOLINT(readability-function-size)
    ros::NodeHandle nh("~");
    IfType testInterface(nh);
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    // values should now be set on parameter server
    {
        std::string verbosity;
        ASSERT_TRUE(nh.getParam("verbosity_param_w_default", verbosity));
        EXPECT_EQ(verbosity, testInterface.verbosity_param_w_default);
    }
    {
        int intInterface = 0;
        ASSERT_TRUE(nh.getParam("int_param_w_default", intInterface));
        ASSERT_EQ(intInterface, testInterface.int_param_w_default);
    }
    {
        double doubleInterface = 0;
        ASSERT_TRUE(nh.getParam("double_param_w_default", doubleInterface));
        EXPECT_EQ(doubleInterface, testInterface.double_param_w_default);
    }
    {
        bool boolInterface = false;
        ASSERT_TRUE(nh.getParam("bool_param_w_default", boolInterface));
        EXPECT_EQ(boolInterface, testInterface.bool_param_w_default);
    }
    // this does not work for long, since
    // on parameter server, long is stored as string with appended "L"
    // workaround:
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("long_param_w_default_int", stringInterface));
        EXPECT_EQ(stringInterface, "1L");
    }
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("long_param_w_default_int_str", stringInterface));
        EXPECT_EQ(stringInterface, "1L");
    }
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("long_param_w_default_long_string", stringInterface));
        EXPECT_EQ(stringInterface, "2147483648L");
    }
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("str_param_w_default", stringInterface));
        EXPECT_EQ(stringInterface, testInterface.str_param_w_default);
    }
    {
        std::vector<int> vectorIntInterface;
        ASSERT_TRUE(nh.getParam("vector_int_param_w_default", vectorIntInterface));
        EXPECT_EQ(vectorIntInterface, testInterface.vector_int_param_w_default);
    }
    {
        std::vector<double> vectorDoubleInterface;
        ASSERT_TRUE(nh.getParam("vector_double_param_w_default", vectorDoubleInterface));
        EXPECT_EQ(vectorDoubleInterface, testInterface.vector_double_param_w_default);
    }
    {
        std::vector<bool> vectorBoolInterface;
        ASSERT_TRUE(nh.getParam("vector_bool_param_w_default", vectorBoolInterface));
        EXPECT_EQ(vectorBoolInterface, testInterface.vector_bool_param_w_default);
    }
    {
        std::vector<std::string> vectorStringInterface;
        ASSERT_TRUE(nh.getParam("vector_string_param_w_default", vectorStringInterface));
        EXPECT_EQ(vectorStringInterface, testInterface.vector_string_param_w_default);
    }
    {
        std::map<std::string, std::string> mapParamWDefault;
        ASSERT_TRUE(nh.getParam("map_param_w_default", mapParamWDefault));
        EXPECT_EQ(mapParamWDefault, testInterface.map_param_w_default);
    }
    {
        int enumIntInterface = 0;
        ASSERT_TRUE(nh.getParam("enum_int_param_w_default", enumIntInterface));
        EXPECT_EQ(enumIntInterface, testInterface.enum_int_param_w_default);
    }
    {
        std::string enumStrInterface;
        ASSERT_TRUE(nh.getParam("enum_str_param_w_default", enumStrInterface));
        EXPECT_EQ(enumStrInterface, testInterface.enum_str_param_w_default);
    }
}

TEST(RosinterfaceHandler, SetParamOnServer) { // NOLINT(readability-function-size)
    ros::NodeHandle nh("~");
    IfType testInterface(nh);
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    testInterface.verbosity_param_w_default = "warning";
    testInterface.int_param_w_default = 2;
    testInterface.double_param_w_default = 2.2;
    testInterface.str_param_w_default = "World Hello";
    testInterface.bool_param_w_default = false;
    testInterface.long_param_w_default_int = 1L;
    testInterface.long_param_w_default_int_str = 1L;
    testInterface.long_param_w_default_long_string = 2147483648L;
    testInterface.vector_int_param_w_default = std::vector<int>{3, 2, 1};
    testInterface.vector_double_param_w_default = std::vector<double>{1.3, 1.2, 1.2};
    testInterface.vector_bool_param_w_default = std::vector<bool>{true, false};
    testInterface.vector_string_param_w_default = std::vector<std::string>{"World", "Hello"};
    testInterface.map_param_w_default = std::map<std::string, std::string>{{"World", "Hello"}};
    testInterface.enum_int_param_w_default = 2;
    testInterface.enum_str_param_w_default = "Two";

    testInterface.toParamServer();

    // values should now be set on parameter server
    {
        std::string verbosity;
        ASSERT_TRUE(nh.getParam("verbosity_param_w_default", verbosity));
        EXPECT_EQ(verbosity, testInterface.verbosity_param_w_default);
    }
    {
        int intInterface = 0;
        ASSERT_TRUE(nh.getParam("int_param_w_default", intInterface));
        ASSERT_EQ(intInterface, testInterface.int_param_w_default);
    }
    {
        double doubleInterface = 0;
        ASSERT_TRUE(nh.getParam("double_param_w_default", doubleInterface));
        EXPECT_EQ(doubleInterface, testInterface.double_param_w_default);
    }
    {
        bool boolInterface = false;
        ASSERT_TRUE(nh.getParam("bool_param_w_default", boolInterface));
        EXPECT_EQ(boolInterface, testInterface.bool_param_w_default);
    }
    // this does not work for long, since
    // on parameter server, long is stored as string with appended "L"
    // workaround:
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("long_param_w_default_int", stringInterface));
        EXPECT_EQ(stringInterface, "1L");
    }
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("long_param_w_default_int_str", stringInterface));
        EXPECT_EQ(stringInterface, "1L");
    }
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("long_param_w_default_long_string", stringInterface));
        EXPECT_EQ(stringInterface, "2147483648L");
    }
    {
        std::string stringInterface;
        ASSERT_TRUE(nh.getParam("str_param_w_default", stringInterface));
        EXPECT_EQ(stringInterface, testInterface.str_param_w_default);
    }
    {
        std::vector<int> vectorIntInterface;
        ASSERT_TRUE(nh.getParam("vector_int_param_w_default", vectorIntInterface));
        EXPECT_EQ(vectorIntInterface, testInterface.vector_int_param_w_default);
    }
    {
        std::vector<double> vectorDoubleInterface;
        ASSERT_TRUE(nh.getParam("vector_double_param_w_default", vectorDoubleInterface));
        EXPECT_EQ(vectorDoubleInterface, testInterface.vector_double_param_w_default);
    }
    {
        std::vector<bool> vectorBoolInterface;
        ASSERT_TRUE(nh.getParam("vector_bool_param_w_default", vectorBoolInterface));
        EXPECT_EQ(vectorBoolInterface, testInterface.vector_bool_param_w_default);
    }
    {
        std::vector<std::string> vectorStringInterface;
        ASSERT_TRUE(nh.getParam("vector_string_param_w_default", vectorStringInterface));
        EXPECT_EQ(vectorStringInterface, testInterface.vector_string_param_w_default);
    }
    {
        std::map<std::string, std::string> mapParamWDefault;
        ASSERT_TRUE(nh.getParam("map_param_w_default", mapParamWDefault));
        EXPECT_EQ(mapParamWDefault, testInterface.map_param_w_default);
    }
    {
        int enumIntInterface = 0;
        ASSERT_TRUE(nh.getParam("enum_int_param_w_default", enumIntInterface));
        EXPECT_EQ(enumIntInterface, testInterface.enum_int_param_w_default);
    }
    {
        std::string enumStrInterface;
        ASSERT_TRUE(nh.getParam("enum_str_param_w_default", enumStrInterface));
        EXPECT_EQ(enumStrInterface, testInterface.enum_str_param_w_default);
    }
}

TEST(RosinterfaceHandler, FromDynamicReconfigure) { // NOLINT(readability-function-size)
    ros::NodeHandle nh("~");
    IfType testInterface(nh);
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)
    testInterface.updater.force_update();

    ConfigType config;
    config.int_param_w_default = 2;
    config.subscriber_w_default_topic = "/in_topic";
    config.subscriber_diag_w_default_topic = "/in_point_topic";
    config.subscriber_public_w_default_topic = "/in_topic";
    config.subscriber_global_w_default_topic = "/in_topic";
    config.publisher_w_default_topic = "/out_topic";
    config.publisher_diag_w_default_topic = "/out_point_topic";
    config.publisher_public_w_default_topic = "/out_topic";
    config.publisher_global_w_default_topic = "/out_topic";
    testInterface.fromConfig(config);

    testInterface.updater.force_update();

    // params
    EXPECT_EQ(testInterface.int_param_w_default, 2);

    // subscriber
    EXPECT_EQ(testInterface.subscriber_w_default->getTopic(), "/in_topic");
    EXPECT_EQ(testInterface.subscriber_diag_w_default->getTopic(), "/in_point_topic");
    EXPECT_EQ(testInterface.subscriber_public_w_default->getTopic(), "/in_topic");
    EXPECT_EQ(testInterface.subscriber_global_w_default->getTopic(), "/in_topic");

    // publisher
    EXPECT_EQ(testInterface.publisher_w_default.getTopic(), "/out_topic");
    EXPECT_EQ(testInterface.publisher_diag_w_default.getTopic(), "/out_point_topic");
    EXPECT_EQ(testInterface.publisher_public_w_default.getTopic(), "/out_topic");
    EXPECT_EQ(testInterface.publisher_global_w_default.getTopic(), "/out_topic");
}
