#include <DefaultsMissingInterface.h>
#include <gtest/gtest.h>

using IfType = rosinterface_handler::DefaultsMissingInterface;
using ConfigType = rosinterface_handler::DefaultsMissingConfig;

TEST(RosinterfaceHandler, DefaultsMissing) {
    IfType testInterface(ros::NodeHandle("~"));
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ASSERT_THROW(testInterface.fromParamServer(), std::runtime_error);
}
