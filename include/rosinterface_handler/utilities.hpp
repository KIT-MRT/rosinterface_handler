#pragma once

#include <limits>
#include <sstream>
#include <string>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/this_node.h>

/// \brief Helper function to test for std::vector
template <typename T>
using IsVector = std::is_same<T, std::vector<typename T::value_type, typename T::allocator_type>>;

/// \brief Helper function to test for std::map
template <typename T>
using IsMap = std::is_same<
    T, std::map<typename T::key_type, typename T::mapped_type, typename T::key_compare, typename T::allocator_type>>;

/// \brief Outstream helper for std:vector
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
    if (!v.empty()) {
        out << '[';
        std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
        out << "\b\b]";
    }
    return out;
}

/// \brief Outstream helper for std:map
template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& stream, const std::map<T1, T2>& map) {
    stream << '{';
    for (auto it = map.begin(); it != map.end(); ++it) {
        stream << (*it).first << " --> " << (*it).second << ", ";
    }
    stream << '}';
    return stream;
}

namespace rosinterface_handler {

/// \brief Retrieve node name
///
/// @param privateNodeHandle The private ROS node handle (i.e.
/// ros::NodeHandle("~") ).
/// @return node name
inline std::string getNodeName(const ros::NodeHandle& privateNodeHandle) {
    std::stringstream tempString(privateNodeHandle.getNamespace());
    std::string name;
    while (std::getline(tempString, name, '/')) {
        ;
    }
    return name;
}

/// \brief Retrieve the parent node handle from a node handle (or /)
///
/// @param privateNodeHandle Any ROS node handle (e.g.
/// ros::NodeHandle("~") ).
/// @return parent namespace or "/"
inline std::string getParentNamespace(const ros::NodeHandle& nodeHandle) {
    const auto& nameSpace = nodeHandle.getNamespace();
    std::string parentNameSpace = nameSpace.substr(0, nameSpace.find_last_of('/'));
    return parentNameSpace.empty() ? "/" : parentNameSpace;
}

/// \brief Sets the logger level according to a standardized parameter name 'verbosity'.
///
/// \param nodeHandle The ROS node handle to search for the parameter 'verbosity'.
// NOLINTNEXTLINE
inline void setLoggerLevel(const ros::NodeHandle& nodeHandle, const std::string& verbosityParam = "verbosity",
                           const std::string& loggerName = "") {

    std::string verbosity;
    if (!nodeHandle.getParam(verbosityParam, verbosity)) {
        verbosity = "warning";
    }

    auto levelRos = ros::console::levels::Info;
    auto validVerbosity = true;
    if (verbosity == "debug") {
        levelRos = ros::console::levels::Debug;
    } else if (verbosity == "info") {
        levelRos = ros::console::levels::Info;
    } else if (verbosity == "warning" || verbosity == "warn") {
        levelRos = ros::console::levels::Warn;
    } else if (verbosity == "error") {
        levelRos = ros::console::levels::Error;
    } else if (verbosity == "fatal") {
        levelRos = ros::console::levels::Fatal;
    } else {
        ROS_WARN_STREAM("Invalid verbosity level specified: " << verbosity << "! Falling back to INFO.");
        validVerbosity = false;
    }
    if (validVerbosity) {
        if (!loggerName.empty() &&
            ros::console::set_logger_level(std::string(ROSCONSOLE_NAME_PREFIX) + "." + loggerName, levelRos)) {
            ros::console::notifyLoggerLevelsChanged();
            ROS_DEBUG_STREAM_NAMED(loggerName, "Verbosity set to " << verbosity);
        }
        // If this is a node, additionally set the default logger, so that ROS_LOG works
        if ((loggerName.empty() || ros::NodeHandle("~").getNamespace() == loggerName) &&
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, levelRos)) {
            ros::console::notifyLoggerLevelsChanged();
            ROS_DEBUG_STREAM("Verbosity set to " << verbosity);
        }
    }
}

/// \brief Show summary about node containing name, namespace, subscribed and advertised topics.
[[deprecated("It doesn't work well on nodelets. Use interfaceObject.showNodeInfo() instead!")]] inline void
showNodeInfo() {

    using namespace ros::this_node;

    std::vector<std::string> subscribedTopics;
    std::vector<std::string> advertisedTopics;
    getSubscribedTopics(subscribedTopics);
    getAdvertisedTopics(advertisedTopics);

    std::ostringstream msgSubscr;
    std::ostringstream msgAdvert;
    for (auto const& t : subscribedTopics) {
        msgSubscr << t << std::endl;
    }
    for (auto const& t : advertisedTopics) {
        msgAdvert << t << std::endl;
    }

    ROS_INFO_STREAM("Started '" << getName() << "' in namespace '" << getNamespace() << "'." << std::endl
                                << "Subscribed topics: " << std::endl
                                << msgSubscr.str() << "Advertised topics: " << std::endl
                                << msgAdvert.str());
}

/// \brief Retrieve the topic to subscribe to (aware of global topic names)
///
/// @param name_space Parent namespace (with trailing "/")
/// @param topic Global or local topic
/// @return name_space + topic or topic if topic is global
inline std::string getTopic(const std::string& nameSpace, const std::string& topic) {
    if (topic.empty() || topic[0] == '/') {
        return topic;
    }
    return nameSpace + topic;
}

/// \brief ExitFunction for rosinterface_handler
inline void exit(const std::string& msg = "Runtime Error in rosinterface handler.") {
    // std::exit(EXIT_FAILURE);
    throw std::runtime_error(msg);
}

/// \brief Set parameter on ROS parameter server
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
inline void setParam(const std::string key, T val) {
    ros::param::set(key, val);
}

/// \brief Get parameter from ROS parameter server quietly
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
// NOLINTNEXTLINE(readability-function-size)
inline bool getParamImpl(const std::string key, T& val) {
    if (!ros::param::has(key)) {
        return false;
    }
    if (!ros::param::get(key, val)) {
        ROS_ERROR_STREAM("Could not retrieve parameter'" << key << "'. Does it have a different type?");
        return false;
    }
    // Param was already retrieved with last if statement.
    return true;
}

/// \brief Get parameter from ROS parameter server or print error
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
// NOLINTNEXTLINE(readability-function-size)
inline bool getParam(const std::string key, T& val) {
    if (!getParamImpl(key, val)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        return false;
    }
    // Param was already retrieved with last if statement.
    return true;
}

/// \brief Get parameter from ROS parameter server or use default value
///
/// If parameter does not exist on server yet, the default value is used and set on server.
/// \param key Parameter name
/// \param val Parameter value
/// \param defaultValue Parameter default value
template <typename T>
// NOLINTNEXTLINE(readability-function-size)
inline bool getParam(const std::string key, T& val, const T& defaultValue) {
    if (!getParamImpl(key, val)) {
        val = defaultValue;
        ros::param::set(key, defaultValue);
        ROS_INFO_STREAM("Parameter '" << key << "' is not defined. Setting default value.");
        return true;
    }
    // Param was already retrieved with last if statement.
    return true;
}

/// \brief Tests that parameter is not set on the parameter server
// NOLINTNEXTLINE(readability-function-size)
inline bool testConstParam(const std::string& key) {
    if (ros::param::has(key)) {
        ROS_WARN_STREAM("Parameter " << key
                                     << "' was set on the parameter server eventhough it was defined to be constant.");
        return false;
    }
    return true;
}

/// \brief Limit parameter to lower bound if parameter is a scalar.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
// NOLINTNEXTLINE(readability-function-size)
inline void testMin(const std::string key, T& val, T min = std::numeric_limits<T>::min()) {
    if (val < min) {
        ROS_WARN_STREAM("Value of " << val << " for " << key
                                    << " is smaller than minimal allowed value. Correcting value to min=" << min);
        val = min;
    }
}

/// \brief Limit parameter to lower bound if parameter is a vector.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMin(const std::string key, std::vector<T>& val, T min = std::numeric_limits<T>::min()) {
    for (auto& v : val) {
        testMin(key, v, min);
    }
}

/// \brief Limit parameter to lower bound if parameter is a map.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename K, typename T>
inline void testMin(const std::string key, std::map<K, T>& val, T min = std::numeric_limits<T>::min()) {
    for (auto& v : val) {
        testMin(key, v.second, min);
    }
}

/// \brief Limit parameter to upper bound if parameter is a scalar.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
// NOLINTNEXTLINE(readability-function-size)
inline void testMax(const std::string key, T& val, T max = std::numeric_limits<T>::max()) {
    if (val > max) {
        ROS_WARN_STREAM("Value of " << val << " for " << key
                                    << " is greater than maximal allowed. Correcting value to max=" << max);
        val = max;
    }
}

/// \brief Limit parameter to upper bound if parameter is a vector.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMax(const std::string key, std::vector<T>& val, T max = std::numeric_limits<T>::max()) {
    for (auto& v : val) {
        testMax(key, v, max);
    }
}

/// \brief Limit parameter to upper bound if parameter is a map.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename K, typename T>
inline void testMax(const std::string key, std::map<K, T>& val, T max = std::numeric_limits<T>::max()) {
    for (auto& v : val) {
        testMax(key, v.second, max);
    }
}

/// \brief Convert at least one argument to a string
/// \tparam Arg Type of required argument
/// \tparam Args Type of additional arguments (optional)
/// \param arg Required argument
/// \param args Additional arguments (optional)
/// \return
template <typename Arg, typename... Args>
inline std::string asString(Arg&& arg, Args&&... Args_) {
    std::ostringstream oss;
    oss << std::forward<Arg>(arg);
    (oss << ... << std::forward<Args>(Args_));
    return oss.str();
}

inline std::string asString(std::string&& arg) {
    return std::move(arg);
}

inline const std::string& asString(const std::string& arg) {
    return arg;
}

} // namespace rosinterface_handler
