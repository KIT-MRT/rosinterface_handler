#pragma once
#include <cstdlib>
#include <mutex>
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <ros/callback_queue.h>
#include <ros/publication.h>
#include <ros/publisher.h>
#include <ros/topic_manager.h>

namespace rosinterface_handler {
namespace detail {
template <typename T>
struct Dereference {
    static inline constexpr decltype(auto) get(const T& t) {
        return t;
    }
};

template <typename T>
struct Dereference<T*> {
    static inline constexpr decltype(auto) get(const T*& t) {
        return *t;
    }
};

template <typename T>
struct Dereference<std::shared_ptr<T>> {
    static constexpr decltype(auto) get(const std::shared_ptr<T>& t) {
        return *t;
    }
};
} // namespace detail
/**
 * @brief Subscriber that only actually subscribes to a topic if someone subscribes to a publisher
 * This is useful to avoid overhead for computing results that no one actually cares for.
 * Because this subscriber internally unsubscribes from a topic, upstream nodes are able to stop
 * publishing useless results as well.
 *
 * The smart subscriber can also be used for synchronized subscription via message_filters::TimeSynchronizer or similar.
 *
 * Set the environment variable NO_SMART_SUBSCRIBE to 1 to disable smart subscriptions.
 *
 * Usage example:
 * @code
 * void messageCallback(const std_msgs::Header::ConstPtr& msg) {
 * // do the work
 * }
 *
 * // subscribe in your main() or nodelet
 * ros::NodeHandle nh;
 * ros::Publisher myPub = nh.advertise<std_msgs::Header>("/output_topic", 5);
 * utils_ros::SmartSubscriber<std_msgs::Header> subscriber(myPub);
 * subscriber.subscribe(nh, "/header_topic", 5);
 * subscriber.addCallback(messageCallback);
 * @endcode
 */
template <class Message>
class SmartSubscriber : public message_filters::Subscriber<Message> {
public:
    using Publishers = std::vector<ros::Publisher>;

    template <typename... PublishersT>
    // NOLINTNEXTLINE(readability-identifier-naming)
    explicit SmartSubscriber(const PublishersT&... trackedPublishers) {
        // check for always-on-mode
        const auto smartSubscribe = std::getenv("NO_SMART_SUBSCRIBE");
        try {
            if (smartSubscribe && std::stoi(smartSubscribe) > 0) {
                setSmart(false);
            }
        } catch (const std::invalid_argument&) {
        }
        ros::SubscriberStatusCallback cb = boost::bind(&SmartSubscriber::subscribeCallback, this);
        callback_ = boost::make_shared<ros::SubscriberCallbacks>(cb, cb, alivePtr_, ros::getGlobalCallbackQueue());

        publisherInfo_.reserve(sizeof...(trackedPublishers));
        using Workaround = int[];
        Workaround{(addPublisher(trackedPublishers), 0)...};
    }
    SmartSubscriber(SmartSubscriber&& rhs) noexcept = delete;
    SmartSubscriber& operator=(SmartSubscriber&& rhs) noexcept = delete;
    SmartSubscriber(const SmartSubscriber& rhs) = delete;
    SmartSubscriber& operator=(const SmartSubscriber& rhs) = delete;
    ~SmartSubscriber() override {
        // void the callback
        alivePtr_.reset(); // makes sure no callbacks are called while destructor is running
        std::lock_guard<std::mutex> m(callbackLock_);
        for (auto& pub : publisherInfo_) {
            removeCallback(pub.topic);
        }
        callback_->disconnect_ = +[](const ros::SingleSubscriberPublisher&) {};
        callback_->connect_ = +[](const ros::SingleSubscriberPublisher&) {};
    }

    /**
     * @brief Subscribe to a topic.
     *
     * Calls the message_filtes::Subscriber's subscribe internally.
     *
     * @param nh The ros::NodeHandle to use to subscribe.
     * @param topic The topic to subscribe to.
     * @param queueSize The subscription queue size
     * @param transportHints The transport hints to pass along
     * @param callbackQueue The callback queue to pass along
     */
    void subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queueSize,
                   const ros::TransportHints& transportHints = ros::TransportHints(),
                   ros::CallbackQueueInterface* callbackQueue = nullptr) override {
        message_filters::Subscriber<Message>::subscribe(nh, topic, queueSize, transportHints, callbackQueue);
        subscribeCallback();
    }

    using message_filters::Subscriber<Message>::subscribe;

    /**
     * @brief Adds a new publisher to monitor
     * @param publisher to look after
     * Requires that the publisher has "getTopic" and a "getNumSubscribers" function.
     * The SmartSubscriber does *not* manage the publisher and keeps a reference to it. If it goes out of scope, there
     * will be trouble.
     */
    template <typename Publisher>
    void addPublisher(const Publisher& publisher) {
        publisherInfo_.push_back({[&]() { return detail::Dereference<Publisher>::get(publisher).getTopic(); },
                                  [&]() { return detail::Dereference<Publisher>::get(publisher).getNumSubscribers(); },
                                  detail::Dereference<Publisher>::get(publisher).getTopic()});
        addCallback(publisherInfo_.back().topic);

        // check for subscribe
        if (!this->getTopic().empty()) {
            subscribeCallback();
        }
    }

    /**
     * @brief stops tracking a publisher.
     * Does nothing if the publisher does not exist.
     * @return true if publisher existed and was removed
     */
    bool removePublisher(const std::string& topic) {
        // remove from vector
        auto found = std::find_if(publisherInfo_.begin(), publisherInfo_.end(),
                                  [&](const auto& pubInfo) { return topic == pubInfo.getTopic(); });
        if (found == publisherInfo_.end()) {
            return false;
        }
        publisherInfo_.erase(found);
        removeCallback(topic);
        return true;
    }

    /**
     * @brief updates the topics of the tracked subscribers
     * This can be necessary if these have changed through a reconfigure request
     */
    // NOLINTNEXTLINE(readability-function-size)
    void updateTopics() {
        for (auto& publisher : publisherInfo_) {
            const auto currTopic = publisher.getTopic();
            if (currTopic != publisher.topic) {
                ROS_DEBUG_STREAM("Publication moved from " << publisher.topic << " to " << currTopic);
                addCallback(currTopic);
                removeCallback(publisher.topic);
                publisher.topic = currTopic;
            }
        }
        subscribeCallback();
    }

    /**
     * @brief returns whether this subsciber is currently subscribed to something
     * @return true if subscribed
     */
    bool isSubscribed() const {
        return bool(this->getSubscriber());
    }

    //! Since this subscriber subscribes automatically, it can not be disabled using unsubscribe(). This function
    //! disables him so that the message callback will no longer be called, no matter how many subscribers there are.
    void disable() {
        disabled_ = true;
        if (isSubscribed()) {
            this->unsubscribe();
        }
    }

    //! Puts a disabled subscriber back into normal mode.
    void enable() {
        disabled_ = false;
        subscribeCallback();
    }

    //! Returns whether this subscriber has been disabled via disable()
    bool isDisabled() const {
        return disabled_;
    }

    /**
     * @brief returns whether this subscriber is currently in smart mode
     * @return true if in smart mode
     * If the subscriber is not in smart mode, it will behave like a normal ros publisher and will always be subscribed
     * unless isDisabled is true.
     */
    bool smart() const {
        return smart_;
    }

    /**
     * @brief enable/disable smart mode
     * @param smart new mode for subscriber
     */
    void setSmart(bool smart) {
        smart_ = smart;
        subscribeCallback();
    }

    /**
     * @brief pass this callback to all non-standard publisher that you have
     * @return subscriber callback of this SmartSubscriber
     */
    const ros::SubscriberCallbacksPtr callback() const {
        return callback_;
    }

    /**
     * @brief checks for new subscribers and subscribes or unsubscribes if anything changed.
     * This function is not supposed to be called actively, it is only here so that you can pass it as callback to any
     * special publisher
     * (like image transport)
     */
    // NOLINTNEXTLINE(readability-function-size)
    void subscribeCallback() {
        std::lock_guard<std::mutex> m(callbackLock_);
        if (disabled_ || !alivePtr_) {
            return;
        }
        const auto subscribed = isSubscribed();
        bool subscribe = !smart() || std::any_of(publisherInfo_.begin(), publisherInfo_.end(),
                                                 [](auto& p) { return p.getNumSubscriber() > 0; });

        if (subscribe && !subscribed) {
            ROS_DEBUG_STREAM("Got new subscribers. Subscribing to " << this->getSubscriber().getTopic());
            this->subscribe();
        }
        if (!subscribe && subscribed) {
            ROS_DEBUG_STREAM("No subscribers found. Unsubscribing from " << this->getSubscriber().getTopic());
            this->unsubscribe();
        }
    }

private:
    // NOLINTNEXTLINE(readability-function-size)
    void addCallback(const std::string& topic) {
        if (topic.empty()) {
            return;
        }
        auto pub = ros::TopicManager::instance()->lookupPublication(topic);
        if (!!pub) {
            pub->addCallbacks(callback_);
        } else {
            ROS_DEBUG_STREAM("Publication not found for topic " << topic);
        }
    }

    void removeCallback(const std::string& topic) {
        auto pub = ros::TopicManager::instance()->lookupPublication(topic);
        if (!!pub) {
            pub->removeCallbacks(callback_);
        }
    }

    struct PublisherInfo {
        std::function<std::string()> getTopic;
        std::function<uint32_t()> getNumSubscriber;
        std::string topic;
    };
    std::vector<PublisherInfo> publisherInfo_;
    boost::shared_ptr<bool> alivePtr_{boost::make_shared<bool>()};
    ros::SubscriberCallbacksPtr callback_;
    std::mutex callbackLock_{};
    bool smart_{true};
    bool disabled_{false};
};

template <class Message>
using SmartSubscriberPtr = std::shared_ptr<SmartSubscriber<Message>>;
} // namespace rosinterface_handler
