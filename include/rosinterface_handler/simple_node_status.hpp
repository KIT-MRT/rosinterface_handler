#pragma once
#include <map>
#include <mutex>
#include <string>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/node_handle.h>

#include "utilities.hpp"

namespace rosinterface_handler {
//! Mirrors diagnostic_updater::DiagnosticStatus, but with strong typing
enum class NodeStatus : std::uint8_t {
    OK = 0,    //!< Everything is fine
    WARN = 1,  //!< Might be a problem
    ERROR = 2, //!< Something is clearly wrong
    STALE = 3, //! State unclear (e.g. during initialization)
};

//! Simplifies the task of reporting the status of a node. All you have to do is initialize this object and from time to
//! time set the current status
class SimpleNodeStatus {
    using StatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;
    struct Status {
        NodeStatus s;
        std::string msg;
        bool operator==(const Status& rhs) const {
            return s == rhs.s && msg == rhs.msg;
        }
        bool operator!=(const Status& rhs) const {
            return !(*this == rhs);
        }
    };

public:
    SimpleNodeStatus(const std::string& statusDescription, const ros::NodeHandle& privNh,
                     diagnostic_updater::Updater& updater)
            // update() only has an effect every second, we need a slightly higher value to get a guaranteed update
            : updateStatus_{privNh.createTimer(ros::Duration(1.01), [&](const auto& /*s*/) { updater_->update(); })},
              updater_{&updater} {
        updater_->add(statusDescription, [this](StatusWrapper& w) { this->getStatus(w); });
        updater_->force_update();
    }

    //! Lightweight way to set or report a new status. The status remains until overwritten by a new status.
    template <typename Arg, typename... Args>
    void set(NodeStatus s, const Arg& arg, const Args&... Args_) {
        bool modified = false;
        {
            std::lock_guard<std::mutex> g{statusMutex_};
            Status newStatus{s, asString(arg, Args_...)};
            modified = status_ != newStatus;
            status_ = newStatus;
        }
        if (s == NodeStatus::ERROR && modified) {
            // new errors are reported asap
            updater_->force_update();
        }
    }

    //! Add/overwrite extra information about the status in form of key/value pairs. The information will be shared
    //! along with the overall node status. It remains until explicitly cleared or overwritten.
    template <typename Arg, typename... Args>
    void info(const std::string& name, const Arg& arg, const Args&... Args_) {
        std::lock_guard<std::mutex> g{statusMutex_};
        extraInfo_[name] = asString(arg, Args_...);
    }

    //! Clears previously set information. Returns true on success.
    bool clearInfo(const std::string& name) {
        std::lock_guard<std::mutex> g{statusMutex_};
        auto it = extraInfo_.find(name);
        if (it != extraInfo_.end()) {
            extraInfo_.erase(it);
            return true;
        }
        return false;
    }

private:
    void getStatus(StatusWrapper& w) const {
        std::lock_guard<std::mutex> g{statusMutex_};
        w.summary(static_cast<std::uint8_t>(status_.s), status_.msg);
        for (const auto& info : extraInfo_) {
            w.add(info.first, info.second);
        }
    }
    ros::Timer updateStatus_;
    diagnostic_updater::Updater* updater_;
    mutable std::mutex statusMutex_;
    Status status_{NodeStatus::STALE, "Initializing"};
    std::map<std::string, std::string> extraInfo_;
};
} // namespace rosinterface_handler
