/*
Delay operation executes in publisher.
Add more timing tracing points.
*/

#include <string>
#include <fstream>
#include <vector>
#include <cfloat>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/latest_time.h>
#include <message_filters/sync_policies/original_latest_time_model.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include "signal.h"
#include "helper.h"

// #define DEBUG
#ifdef DEBUG
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif


using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::placeholders::_8;
using std::placeholders::_9;

namespace synchronizer
{


class SubscriberTopic2
    : public rclcpp::Node
{
    public:
    SubscriberTopic2(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic2"),
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic2::callback1, this, _1));
        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic2::callback2, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic2::mdl_callback, this, _1, _2));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic2::alg_callback, this, _1, _2));

    }

    ~SubscriberTopic2()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<2 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets

    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
    
};

class SubscriberTopic3
    : public rclcpp::Node
{
    public:
    SubscriberTopic3(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic3"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic3::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic3::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic3::callback3, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic3::mdl_callback, this, _1, _2, _3));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic3::alg_callback, this, _1, _2, _3));

    }

    ~SubscriberTopic3()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<3 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<3 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};

class SubscriberTopic4
    : public rclcpp::Node
{

    public:
    SubscriberTopic4(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic4"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic4::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic4::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic4::callback3, this, _1));
        
        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic4::callback4, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic4::mdl_callback, this, _1, _2, _3, _4));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic4::alg_callback, this, _1, _2, _3, _4));

    }

    ~SubscriberTopic4()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<4 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<4 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<3>(msg);
        alg_sync_.add<3>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};

class SubscriberTopic5
    : public rclcpp::Node
{
    public:
    SubscriberTopic5(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic5"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic5::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic5::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic5::callback3, this, _1));
        
        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic5::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic5::callback5, this, _1));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic5::mdl_callback, this, _1, _2, _3, _4, _5));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic5::alg_callback, this, _1, _2, _3, _4, _5));

    }

    ~SubscriberTopic5()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<5 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<5 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<3>(msg);
        alg_sync_.add<3>(msg);
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<4>(msg);
        alg_sync_.add<4>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic5_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};

class SubscriberTopic6
    : public rclcpp::Node
{
    public:
    SubscriberTopic6(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic6"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic6::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic6::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic6::callback3, this, _1));
        
        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic6::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic6::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic6::callback6, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic6::mdl_callback, this, _1, _2, _3, _4, _5, _6));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic6::alg_callback, this, _1, _2, _3, _4, _5, _6));

    }

    ~SubscriberTopic6()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        debug_printf("mdl published !\n");
        debug_printf("\tmsg1 %9.9lf at %9.9lf\n", (double)msg1->header.stamp.sec+(double)msg1->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg2 %9.9lf at %9.9lf\n", (double)msg2->header.stamp.sec+(double)msg2->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg3 %9.9lf at %9.9lf\n", (double)msg3->header.stamp.sec+(double)msg3->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg4 %9.9lf at %9.9lf\n", (double)msg4->header.stamp.sec+(double)msg4->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg5 %9.9lf at %9.9lf\n", (double)msg5->header.stamp.sec+(double)msg5->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg6 %9.9lf at %9.9lf\n", (double)msg6->header.stamp.sec+(double)msg6->header.stamp.nanosec/1e9, this->now().seconds());
        
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<6 ; ++j){
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);
                }
                    
            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        debug_printf("alg published !\n");
        debug_printf("\tmsg1 %9.9lf at %9.9lf\n", (double)msg1->header.stamp.sec+(double)msg1->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg2 %9.9lf at %9.9lf\n", (double)msg2->header.stamp.sec+(double)msg2->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg3 %9.9lf at %9.9lf\n", (double)msg3->header.stamp.sec+(double)msg3->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg4 %9.9lf at %9.9lf\n", (double)msg4->header.stamp.sec+(double)msg4->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg5 %9.9lf at %9.9lf\n", (double)msg5->header.stamp.sec+(double)msg5->header.stamp.nanosec/1e9, this->now().seconds());
        debug_printf("\tmsg6 %9.9lf at %9.9lf\n", (double)msg6->header.stamp.sec+(double)msg6->header.stamp.nanosec/1e9, this->now().seconds());
        
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<6 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        debug_printf("sub1 %9.9lf at %9.9lf\n", (double)msg->header.stamp.sec+(double)msg->header.stamp.nanosec/1e9, this->now().seconds());
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        debug_printf("sub2 %9.9lf at %9.9lf\n", (double)msg->header.stamp.sec+(double)msg->header.stamp.nanosec/1e9, this->now().seconds());
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        debug_printf("sub3 %9.9lf at %9.9lf\n", (double)msg->header.stamp.sec+(double)msg->header.stamp.nanosec/1e9, this->now().seconds());
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        debug_printf("sub4 %9.9lf at %9.9lf\n", (double)msg->header.stamp.sec+(double)msg->header.stamp.nanosec/1e9, this->now().seconds());
        mdl_sync_.add<3>(msg);
        alg_sync_.add<3>(msg);
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        debug_printf("sub5 %9.9lf at %9.9lf\n", (double)msg->header.stamp.sec+(double)msg->header.stamp.nanosec/1e9, this->now().seconds());
        mdl_sync_.add<4>(msg);
        alg_sync_.add<4>(msg);
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        debug_printf("sub6 %9.9lf at %9.9lf\n", (double)msg->header.stamp.sec+(double)msg->header.stamp.nanosec/1e9, this->now().seconds());
        mdl_sync_.add<5>(msg);
        alg_sync_.add<5>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic5_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic6_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};

class SubscriberTopic7
    : public rclcpp::Node
{
    public:
    SubscriberTopic7(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic7"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic7::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic7::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic7::callback3, this, _1));
        
        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic7::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic7::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic7::callback6, this, _1));
                                                                        
        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic7::callback7, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic7::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic7::alg_callback, this, _1, _2, _3, _4, _5, _6, _7));

    }

    ~SubscriberTopic7()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<7 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        output_set.push_back(msg7);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<7 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        output_set.push_back(msg7);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<3>(msg);
        alg_sync_.add<3>(msg);
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<4>(msg);
        alg_sync_.add<4>(msg);
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<5>(msg);
        alg_sync_.add<5>(msg);
    }

    void callback7(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<6>(msg);
        alg_sync_.add<6>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic5_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic6_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic7_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};

class SubscriberTopic8
    : public rclcpp::Node
{
    public:
    SubscriberTopic8(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic8"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic8::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic8::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic8::callback3, this, _1));
        
        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic8::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic8::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic8::callback6, this, _1));
                                                                        
        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic8::callback7, this, _1));
                                                                        
        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic8::callback8, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic8::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic8::alg_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

    }

    ~SubscriberTopic8()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<8 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        output_set.push_back(msg7);
        output_set.push_back(msg8);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<8 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        output_set.push_back(msg7);
        output_set.push_back(msg8);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<3>(msg);
        alg_sync_.add<3>(msg);
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<4>(msg);
        alg_sync_.add<4>(msg);
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<5>(msg);
        alg_sync_.add<5>(msg);
    }

    void callback7(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<6>(msg);
        alg_sync_.add<6>(msg);
    }

    void callback8(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<7>(msg);
        alg_sync_.add<7>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic5_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic6_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic7_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic8_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};

class SubscriberTopic9
    : public rclcpp::Node
{
    public:
    SubscriberTopic9(rclcpp::Clock::SharedPtr clock) :
        Node("subscriber_topic9"), 
        alg_sync_(clock), mdl_sync_(clock),
        count_alg_(0), count_mdl_(0)
    {
        this->declare_parameter("num_published_set", 2000);

        this->get_parameter("num_published_set", num_published_set_);

        // RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic9::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic9::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic9::callback3, this, _1));
        
        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic9::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic9::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic9::callback6, this, _1));
                                                                        
        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic9::callback7, this, _1));
                                                                        
        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback8, this, _1));
                                                                        
        topic9_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic9", 300, 
                                                                        std::bind(&SubscriberTopic9::callback9, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic9::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));
        alg_sync_.registerCallback(std::bind(&SubscriberTopic9::alg_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));

    }

    ~SubscriberTopic9()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("mdl has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<9 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        output_set.push_back(msg7);
        output_set.push_back(msg8);
        output_set.push_back(msg9);
        mdl_ouput_set_.push_back(output_set);

        count_mdl_++;
    }

    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8
                        , const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        if(count_mdl_ >= num_published_set_ && count_alg_ >= num_published_set_)
        {
            debug_printf("alg has got enough published sets !\n");

            for (int i = 0; i<num_published_set_ ; ++i)
                for (int j = 0 ; j<9 ; ++j)
                    assert(mdl_ouput_set_[j] == alg_ouput_set_[j]);

            kill(getpid(),SIGINT);
            return;
        }

        std::vector<sensor_msgs::msg::JointState::ConstSharedPtr> output_set;
        output_set.push_back(msg1);
        output_set.push_back(msg2);
        output_set.push_back(msg3);
        output_set.push_back(msg4);
        output_set.push_back(msg5);
        output_set.push_back(msg6);
        output_set.push_back(msg7);
        output_set.push_back(msg8);
        output_set.push_back(msg9);
        alg_ouput_set_.push_back(output_set);

        count_alg_++;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<0>(msg);
        alg_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<1>(msg);
        alg_sync_.add<1>(msg);
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<2>(msg);
        alg_sync_.add<2>(msg);
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<3>(msg);
        alg_sync_.add<3>(msg);
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<4>(msg);
        alg_sync_.add<4>(msg);
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<5>(msg);
        alg_sync_.add<5>(msg);
    }

    void callback7(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<6>(msg);
        alg_sync_.add<6>(msg);
    }

    void callback8(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<7>(msg);
        alg_sync_.add<7>(msg);
    }

    void callback9(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        mdl_sync_.add<8>(msg);
        alg_sync_.add<8>(msg);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic5_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic6_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic7_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic8_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic9_sub_;

    typedef Synchronizer<sync_policies::LatestTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                    , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState
                                                        , sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> mdl_ouput_set_;
    std::vector<std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>> alg_ouput_set_;
};
}


