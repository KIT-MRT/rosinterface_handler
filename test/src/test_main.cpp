#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rosinterface_handler_test");
    testing::FLAGS_gtest_catch_exceptions = false;
    // The async spinner lets you publish and receive messages during the tests,
    // no need to call spinOnce()
    ros::AsyncSpinner spinner(2);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
