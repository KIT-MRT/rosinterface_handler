#pragma once
#include <mutex>
#include <string>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/node_handle.h>

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
            : updateStatus_{privNh.createTimer(ros::Duration(1.01), [&](const auto& /*s*/) { updater_->update(); })},
              updater_{&updater} {
        updater_->add(statusDescription, [this](StatusWrapper& w) { this->getStatus(w); });
        updater_->force_update();
    }

    //! Leightweight way to set or report a new status
    void set(NodeStatus s, const std::string& msg) {
        bool modified = false;
        {
            std::lock_guard<std::mutex> g{statusMutex_};
            Status newStatus{s, msg};
            modified = status_ != newStatus;
            status_ = newStatus;
        }
        if (s == NodeStatus::ERROR && modified) {
            // new errors are reported asap
            updater_->force_update();
        }
    }

private:
    void getStatus(StatusWrapper& w) const {
        std::lock_guard<std::mutex> g{statusMutex_};
        w.summary(static_cast<std::uint8_t>(status_.s), status_.msg);
    }
    ros::Timer updateStatus_;
    diagnostic_updater::Updater* updater_;
    mutable std::mutex statusMutex_;
    Status status_{NodeStatus::STALE, "Initializing"};
};
} // namespace rosinterface_handler
