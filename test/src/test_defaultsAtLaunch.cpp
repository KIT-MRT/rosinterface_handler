#include <DefaultsAtLaunchInterface.h>
#include <gtest/gtest.h>

using IfType = rosinterface_handler::DefaultsAtLaunchInterface;
using ConfigType = rosinterface_handler::DefaultsAtLaunchConfig;

TEST(RosinterfaceHandler, DefaultsAtLaunch) { // NOLINT(readability-function-size)
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    ASSERT_EQ("info", testInterface.verbosity_param_wo_default);

    ASSERT_EQ(1, testInterface.int_param_wo_default);
    ASSERT_DOUBLE_EQ(1.1, testInterface.double_param_wo_default);
    ASSERT_EQ("Hello World", testInterface.str_param_wo_default);
    ASSERT_EQ(true, testInterface.bool_param_wo_default);
    ASSERT_EQ(1L, testInterface.long_param_wo_default_int);
    ASSERT_EQ(1L, testInterface.long_param_wo_default_int_str);
    ASSERT_EQ(2147483648L, testInterface.long_param_wo_default_long_str);

    ASSERT_EQ(std::vector<int>({1, 2, 3}), testInterface.vector_int_param_wo_default);
    ASSERT_EQ(std::vector<double>({1.1, 1.2, 1.3}), testInterface.vector_double_param_wo_default);
    ASSERT_EQ(std::vector<bool>({false, true}), testInterface.vector_bool_param_wo_default);
    ASSERT_EQ(std::vector<std::string>({"Hello", "World"}), testInterface.vector_string_param_wo_default);

    std::map<std::string, std::string> tmp{{"Hello", "World"}};
    ASSERT_EQ(tmp, testInterface.map_param_wo_default);

    ASSERT_EQ(1, testInterface.enum_int_param_wo_default);
    ASSERT_EQ("Two", testInterface.enum_str_param_wo_default);
}

TEST(RosinterfaceHandler, AtLaunchSubscriber) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    ASSERT_TRUE(!!testInterface.subscriber_wo_default);
    ASSERT_EQ(testInterface.subscriber_wo_default->getTopic(), "/test/rosinterface_handler_test/in_topic");

    ASSERT_TRUE(!!testInterface.subscriber_public_wo_default);
    ASSERT_EQ(testInterface.subscriber_public_wo_default->getTopic(), "/test/in_topic");

    ASSERT_TRUE(!!testInterface.subscriber_global_wo_default);
    ASSERT_EQ(testInterface.subscriber_global_wo_default->getTopic(), "/in_topic");
}

TEST(RosinterfaceHandler, AtLaunchPublisher) {
    IfType testInterface(ros::NodeHandle("~"));
    ASSERT_NO_THROW(testInterface.fromParamServer()); // NOLINT(cppcoreguidelines-avoid-goto)

    ASSERT_EQ(testInterface.publisher_wo_default.getTopic(), "/test/rosinterface_handler_test/out_topic");
    ASSERT_EQ(testInterface.publisher_public_wo_default.getTopic(), "/test/out_topic");
    ASSERT_EQ(testInterface.publisher_global_wo_default.getTopic(), "/out_topic");
}
