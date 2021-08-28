cmake_minimum_required(VERSION 3.5.0)
project(rosinterface_handler)

find_package(catkin REQUIRED COMPONENTS message_generation)

# export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
	# set compiler flags
    find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure diagnostic_updater rostest roscpp message_filters std_msgs)
    set(ROSINTERFACE_HANDLER_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR}/cmake)
    include(cmake/rosinterface_handler-macros.cmake)
    file(GLOB PROJECT_TEST_FILES_INTERFACE RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/cfg/*.rosif")
    generate_ros_interface_files(${PROJECT_TEST_FILES_INTERFACE})
endif()

add_service_files(FILES GetTopic.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS catkin
        CFG_EXTRAS rosinterface_handler-extras.cmake
)

install(
        DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

install(
        DIRECTORY templates
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        DIRECTORY include/rosinterface_handler/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/generate_yaml
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    file(GLOB PROJECT_TEST_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/src/*.cpp")
    set(TEST_TARGET_NAME "rosinterface_handler_test")
    add_rostest_gtest(${TEST_TARGET_NAME} test/launch/rosinterface_handler.test ${PROJECT_TEST_FILES_SRC})
    add_rostest(test/launch/rosinterface_handler_topic_server.test DEPENDENCIES ${TEST_TARGET_NAME})
    add_rostest(test/launch/rosinterface_handler_python.test DEPENDENCIES ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_rostest(test/launch/rosinterface_handler_topic_server_python.test DEPENDENCIES ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    find_package(tf2_ros REQUIRED)
    find_package(image_transport REQUIRED)
    target_link_libraries(${TEST_TARGET_NAME} ${catkin_LIBRARIES} ${tf2_ros_LIBRARIES} ${image_transport_LIBRARIES} gtest)
    target_include_directories(${TEST_TARGET_NAME} PUBLIC include)
    target_include_directories(${TEST_TARGET_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS})
    # put dir of generated headers at the front. dynamic_reconfigure messes this up (see #173).
    target_include_directories(${TEST_TARGET_NAME} SYSTEM BEFORE PUBLIC ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    add_dependencies(${TEST_TARGET_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD 17)
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
endif()
