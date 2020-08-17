#pragma once
#define IF_HANDLER_DIAGNOSTICS_INCLUDED
#include <memory>
#include <diagnostic_updater/publisher.h>
#include <message_filters/subscriber.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>

namespace rosinterface_handler {
//! TopicDiagnostic does not clean up after itself. This wrapper does just that.
class TopicDiagnosticWrapper {
public:
    TopicDiagnosticWrapper(std::string name, diagnostic_updater::Updater& diag,
                           const diagnostic_updater::FrequencyStatusParam& freq,
                           const diagnostic_updater::TimeStampStatusParam& stamp)
            : updater_{diag}, diag_(std::move(name), diag, freq, stamp) {
    }
    TopicDiagnosticWrapper(TopicDiagnosticWrapper&& rhs) noexcept = delete;
    TopicDiagnosticWrapper& operator=(TopicDiagnosticWrapper&& rhs) noexcept = delete;
    TopicDiagnosticWrapper(const TopicDiagnosticWrapper& rhs) = delete;
    TopicDiagnosticWrapper& operator=(const TopicDiagnosticWrapper& rhs) = delete;

    ~TopicDiagnosticWrapper() {
        updater_.removeByName(diag_.getName()); // this is the line we actually need..
    }

    void tick() {
        diag_.tick();
    }

    void tick(const ros::Time& stamp) {
        diag_.tick(stamp);
    }

    const std::string& name() {
        return diag_.getName();
    }

private:
    diagnostic_updater::Updater& updater_;
    diagnostic_updater::TopicDiagnostic diag_;
};

//! Like a message_filters::Subscriber, but also manages diagnostics.
template <typename MsgT, typename SubscriberBase = message_filters::Subscriber<MsgT>>
class DiagnosedSubscriber : public SubscriberBase {
    static_assert(ros::message_traits::HasHeader<MsgT>::value,
                  "DiagnosedSubscriber can only be used on messages with a header!");
    using SubscriberT = SubscriberBase;
    using MsgPtrT = boost::shared_ptr<const MsgT>;

public:
    template <typename... Args>
    // NOLINTNEXTLINE(readability-identifier-naming)
    explicit DiagnosedSubscriber(diagnostic_updater::Updater& updater, Args&&... args)
            : SubscriberBase(std::forward<Args>(args)...), updater_{updater} {
        SubscriberT::registerCallback([this](const MsgPtrT& msg) { this->onMessage(msg); });
    }

    DiagnosedSubscriber& minFrequency(double minFrequency) {
        this->minFreq_ = minFrequency;
        return *this;
    }
    DiagnosedSubscriber& maxTimeDelay(double maxTimeDelay) {
        this->maxTimeDelay_ = maxTimeDelay;
        initDiagnostic(this->getTopic());
        return *this;
    }

    void subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queueSize,
                   const ros::TransportHints& transportHints = ros::TransportHints(),
                   ros::CallbackQueueInterface* callbackQueue = nullptr) override {
        SubscriberT::subscribe(nh, topic, queueSize, transportHints, callbackQueue);
        initDiagnostic(topic);
    }

    void subscribe() override {
        SubscriberT::subscribe();
        initDiagnostic(this->getTopic());
    }

    void unsubscribe() override {
        SubscriberT::unsubscribe();
        initDiagnostic("");
    }

private:
    void onMessage(const MsgPtrT& msg) {
        diagnostic_->tick(msg->header.stamp);
    }

    void initDiagnostic(const std::string& name) {
        diagnostic_.reset();
        if (name.empty()) {
            return;
        }
        using namespace diagnostic_updater;
        // we allow messages from the near future because rosbag play sometimes creates those
        constexpr double MinTimeDelay = -0.01;
        diagnostic_ = std::make_unique<TopicDiagnosticWrapper>(name + " subscriber", updater_,
                                                               FrequencyStatusParam(&minFreq_, &maxFreq_, 0),
                                                               TimeStampStatusParam(MinTimeDelay, maxTimeDelay_));
    }
    double minFreq_{0.};
    double maxFreq_{std::numeric_limits<double>::infinity()};
    double maxTimeDelay_{0.};
    diagnostic_updater::Updater& updater_;
    std::unique_ptr<TopicDiagnosticWrapper> diagnostic_;
};

//! Similar to diagnostic_updater::DiagnosedPublisher, but with less segfaults and a simpler interface. Low frequency
//! and delay are only treated as warnings/errors if there are actually subscribers on the advertised topic.
template <typename MsgT>
class DiagnosedPublisher {
    static_assert(ros::message_traits::HasHeader<MsgT>::value,
                  "DiagnosedPublisher can only be used on messages with a header!");
    using Publisher = diagnostic_updater::DiagnosedPublisher<const MsgT>;
    class PublisherData {
    public:
        PublisherData(diagnostic_updater::Updater& updater, const ros::Publisher& publisher, double minFreq,
                      double maxTimeDelay)
                : minFreq_{minFreq}, updater_{&updater},
                  publisher_{publisher, updater, diagnostic_updater::FrequencyStatusParam(&minFreq_, &maxFreq_, 0.),
                             diagnostic_updater::TimeStampStatusParam(0., maxTimeDelay)} {
            // We want to control the result of the updater ourselves. Therefore we remove the callback registered by
            // the Publisher and replace it with our own callback.
            auto name = publisher_.getName();
            updater.removeByName(name);
            updater.add(name, [&](diagnostic_updater::DiagnosticStatusWrapper& msg) {
                publisher_.run(msg);
                if (getNumSubscribers() == 0) {
                    msg.level = diagnostic_msgs::DiagnosticStatus::OK;
                    msg.message = "No subscribers; " + msg.message;
                }
            });
        }
        ~PublisherData() {
            updater_->removeByName(publisher_.getName());
        }
        PublisherData() noexcept = delete;
        PublisherData(PublisherData&& rhs) noexcept = delete;
        PublisherData& operator=(PublisherData&& rhs) noexcept = delete;
        PublisherData(const PublisherData& rhs) = delete;
        PublisherData& operator=(const PublisherData& rhs) = delete;

        template <typename T>
        void publish(const T& msg) {
            publisher_.publish(msg);
        }

        uint32_t getNumSubscribers() const {
            return publisher_.getPublisher().getNumSubscribers();
        }
        ros::Publisher publisher() const {
            return publisher_.getPublisher();
        }

    private:
        double minFreq_{0.};
        double maxFreq_{1.e8};
        diagnostic_updater::Updater* updater_{nullptr};
        Publisher publisher_;
    };

public:
    explicit DiagnosedPublisher(diagnostic_updater::Updater& updater) : updater_{&updater} {
    }
    DiagnosedPublisher() noexcept = default;
    DiagnosedPublisher(DiagnosedPublisher&& rhs) noexcept = default;
    DiagnosedPublisher& operator=(DiagnosedPublisher&& rhs) noexcept = default;
    DiagnosedPublisher(const DiagnosedPublisher& rhs) = default;
    DiagnosedPublisher& operator=(const DiagnosedPublisher& rhs) = default;
    ~DiagnosedPublisher() noexcept = default;

    DiagnosedPublisher& operator=(const ros::Publisher& publisher) {
        reset(publisher);
        return *this;
    }

    void publish(const boost::shared_ptr<const MsgT>& message) {
        if (!!publisherData_) {
            publisherData_->publish(message);
        }
    }

    void publish(const MsgT& message) {
        if (!!publisherData_) {
            publisherData_->publish(message);
        }
    }

    DiagnosedPublisher& minFrequency(double minFrequency) {
        minFreq_ = minFrequency;
        if (!!publisherData_) {
            reset(publisherData_->publisher());
        }
        return *this;
    }

    DiagnosedPublisher& maxTimeDelay(double maxTimeDelay) {
        maxTimeDelay_ = maxTimeDelay;
        if (!!publisherData_) {
            reset(publisherData_->publisher());
        }
        return *this;
    }

    ros::Publisher publisher() const {
        if (!!publisherData_) {
            return publisherData_->publisher();
        }
        return ros::Publisher();
    }

    std::string getTopic() const {
        return publisher().getTopic();
    }

    uint32_t getNumSubscribers() const {
        return !publisherData_ ? uint32_t() : publisherData_->getNumSubscribers();
    }

private:
    void reset(const ros::Publisher& publisher) {
        publisherData_ = std::make_shared<PublisherData>(*updater_, publisher, minFreq_, maxTimeDelay_);
    }
    double minFreq_{0.};
    double maxTimeDelay_{0.};
    diagnostic_updater::Updater* updater_{nullptr};
    std::shared_ptr<PublisherData> publisherData_;
};
} // namespace rosinterface_handler
