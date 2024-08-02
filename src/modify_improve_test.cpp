#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h> // include the header file for sleep()

#include <string>
#include <vector>

#include "npublisher.h"
#include "rclcpp/time_source.hpp"

#include <string>
#include <fstream>
#include <vector>
#include <cfloat>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/latest_time_model.h>
#include <message_filters/sync_policies/original_latest_time_model.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include "signal.h"
#include "helper.h"

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
using namespace std;

using namespace std::chrono_literals;


class SubscriberTopic6 : public rclcpp::Node {
    ofstream& outfile_;
    public:
    SubscriberTopic6(std::ofstream& outfile) :
        Node("subscriber_topic6"), outfile_(outfile), mdl_sync_(), count_mdl_(0), count_omdl_(0)
    {
        num_published_set_ = 2000;
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
        omdl_sync_.registerCallback(std::bind(&SubscriberTopic6::omdl_callback, this, _1, _2, _3, _4, _5, _6));
    }

    ~SubscriberTopic6()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, 
                    const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        if (count_mdl_ == num_published_set_) return;

        rclcpp::Time output_time = (rclcpp::Time)msg1->timing.arrival_time;
        if ((rclcpp::Time)msg2->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg2->timing.arrival_time;
        if ((rclcpp::Time)msg3->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg3->timing.arrival_time;
        if ((rclcpp::Time)msg4->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg4->timing.arrival_time;
        if ((rclcpp::Time)msg5->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg5->timing.arrival_time;
        if ((rclcpp::Time)msg6->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg6->timing.arrival_time;
        
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }
            
            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - previous_arrival_time5_).seconds();
            if (reaction_latency5 > max_reaction_latency5_)
            {
                max_reaction_latency5_ = reaction_latency5;
            }

            double reaction_latency6 = (output_time - previous_arrival_time6_).seconds();
            if (reaction_latency6 > max_reaction_latency6_)
            {
                max_reaction_latency6_ = reaction_latency6;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;
        previous_arrival_time5_ = msg5->timing.arrival_time;
        previous_arrival_time6_ = msg6->timing.arrival_time;

        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;
        rclcpp::Time start_time6 = msg6->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();
        double latency6 = (output_time - start_time6).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }
        
        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }

        if (latency5 > max_passing_latency5_)
        {
            max_passing_latency5_ = latency5;
        }

        if (latency6 > max_passing_latency6_)
        {
            max_passing_latency6_ = latency6;
        }

        double timestamp1 = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double timestamp2 = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double timestamp3 = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double timestamp4 = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double timestamp5 = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double timestamp6 = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double earliest = timestamp1, latest = timestamp1;
        if (timestamp2 < earliest)
            earliest = timestamp2;
        if (timestamp2 > latest)
            latest = timestamp2;
        if (timestamp3 < earliest)
            earliest = timestamp3;
        if (timestamp3 > latest)
            latest = timestamp3;
        if (timestamp4 < earliest)
            earliest = timestamp4;
        if (timestamp4 > latest)
            latest = timestamp4;
        if (timestamp5 < earliest)
            earliest = timestamp5;
        if (timestamp5 > latest)
            latest = timestamp5;
        if (timestamp6 < earliest)
            earliest = timestamp6;
        if (timestamp6 > latest)
            latest = timestamp6;
        if (latest - earliest > max_time_disparity_)
            max_time_disparity_ = latest - earliest;
    }

    void omdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, const sensor_msgs::msg::JointState::ConstSharedPtr& msg3,
                    const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        rclcpp::Time output_time = (rclcpp::Time)msg1->timing.arrival_time;
        if ((rclcpp::Time)msg2->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg2->timing.arrival_time;
        if ((rclcpp::Time)msg3->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg3->timing.arrival_time;
        if ((rclcpp::Time)msg4->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg4->timing.arrival_time;
        if ((rclcpp::Time)msg5->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg5->timing.arrival_time;
        if ((rclcpp::Time)msg6->timing.arrival_time > output_time)
            output_time = (rclcpp::Time)msg6->timing.arrival_time;

        if(count_omdl_ == num_published_set_ && count_mdl_ == num_published_set_)
        {
            double omax_r = omax_reaction_latency1_;
            double max_r = max_reaction_latency1_;
            if (omax_reaction_latency2_ > omax_r)
                omax_r = omax_reaction_latency2_;
            if (max_reaction_latency2_ > max_r)
                max_r = max_reaction_latency2_;
            if (omax_reaction_latency3_ > omax_r)
                omax_r = omax_reaction_latency3_;
            if (max_reaction_latency3_ > max_r)
                max_r = max_reaction_latency3_;
            if (omax_reaction_latency4_ > omax_r)
                omax_r = omax_reaction_latency4_;
            if (max_reaction_latency4_ > max_r)
                max_r = max_reaction_latency4_;
            if (omax_reaction_latency5_ > omax_r)
                omax_r = omax_reaction_latency5_;
            if (max_reaction_latency5_ > max_r)
                max_r = max_reaction_latency5_;
            if (omax_reaction_latency6_ > omax_r)
                omax_r = omax_reaction_latency6_;
            if (max_reaction_latency6_ > max_r)
                max_r = max_reaction_latency6_;
            double omax_p = omax_passing_latency1_;
            double max_p = max_passing_latency1_;
            if (omax_passing_latency2_ > omax_p)
                omax_p = omax_passing_latency2_;
            if (max_passing_latency2_ > max_p)
                max_p = max_passing_latency2_;
            if (omax_passing_latency3_ > omax_p)
                omax_p = omax_passing_latency3_;
            if (max_passing_latency3_ > max_p)
                max_p = max_passing_latency3_;
            if (omax_passing_latency4_ > omax_p)
                omax_p = omax_passing_latency4_;
            if (max_passing_latency4_ > max_p)
                max_p = max_passing_latency4_;
            if (omax_passing_latency5_ > omax_p)
                omax_p = omax_passing_latency5_;
            if (max_passing_latency5_ > max_p)
                max_p = max_passing_latency5_;
            if (omax_passing_latency6_ > omax_p)
                omax_p = omax_passing_latency6_;
            if (max_passing_latency6_ > max_p)
                max_p = max_passing_latency6_;

            // cout << "------------ pub num: old " << count_omdl_ << " new " << count_mdl_ << "-----------" << endl;
            // cout << "          \told\t\tnew\t\tover" << endl;
            // cout << "disparity \t" << fixed << setprecision(6) << omax_time_disparity_*1000 << "\t" << max_time_disparity_*1000 << "\t" << omax_time_disparity_/max_time_disparity_ << endl;
            // cout << "passing   \t" << fixed << setprecision(6) << omax_p*1000 << "\t" << max_p*1000 << "\t" << omax_p/max_p << endl;
            // cout << "reaction  \t" << fixed << setprecision(6) << omax_r*1000 << "\t" << max_r*1000 << "\t" << omax_r/max_r << endl;
            // cout << endl;
            outfile_ << max_time_disparity_/omax_time_disparity_ << ' ' << max_p/omax_p << ' ' << max_r/omax_r << endl;

            kill(getpid(),SIGINT);
        }

        if (count_omdl_ == num_published_set_) return;
        count_omdl_++;

        if (count_omdl_ > 1)
        {
            double reaction_latency1 = (output_time - oprevious_arrival_time1_).seconds();
            if (reaction_latency1 > omax_reaction_latency1_)
            {
                omax_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - oprevious_arrival_time2_).seconds();
            if (reaction_latency2 > omax_reaction_latency2_)
            {
                omax_reaction_latency2_ = reaction_latency2;
            }
            
            double reaction_latency3 = (output_time - oprevious_arrival_time3_).seconds();
            if (reaction_latency3 > omax_reaction_latency3_)
            {
                omax_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - oprevious_arrival_time4_).seconds();
            if (reaction_latency4 > omax_reaction_latency4_)
            {
                omax_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - oprevious_arrival_time5_).seconds();
            if (reaction_latency5 > omax_reaction_latency5_)
            {
                omax_reaction_latency5_ = reaction_latency5;
            }

            double reaction_latency6 = (output_time - oprevious_arrival_time6_).seconds();
            if (reaction_latency6 > omax_reaction_latency6_)
            {
                omax_reaction_latency6_ = reaction_latency6;
            }
        }

        oprevious_arrival_time1_ = msg1->timing.arrival_time;
        oprevious_arrival_time2_ = msg2->timing.arrival_time;
        oprevious_arrival_time3_ = msg3->timing.arrival_time;
        oprevious_arrival_time4_ = msg4->timing.arrival_time;
        oprevious_arrival_time5_ = msg5->timing.arrival_time;
        oprevious_arrival_time6_ = msg6->timing.arrival_time;
        
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;
        rclcpp::Time start_time6 = msg6->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();
        double latency6 = (output_time - start_time6).seconds();

        if (latency1 > omax_passing_latency1_)
        {
            omax_passing_latency1_ = latency1;
        }

        if (latency2 > omax_passing_latency2_)
        {
            omax_passing_latency2_ = latency2;
        }
        
        if (latency3 > omax_passing_latency3_)
        {
            omax_passing_latency3_ = latency3;
        }

        if (latency4 > omax_passing_latency4_)
        {
            omax_passing_latency4_ = latency4;
        }

        if (latency5 > omax_passing_latency5_)
        {
            omax_passing_latency5_ = latency5;
        }

        if (latency6 > omax_passing_latency6_)
        {
            omax_passing_latency6_ = latency6;
        }

        double timestamp1 = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double timestamp2 = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double timestamp3 = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double timestamp4 = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double timestamp5 = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double timestamp6 = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double earliest = timestamp1, latest = timestamp1;
        if (timestamp2 < earliest)
            earliest = timestamp2;
        if (timestamp2 > latest)
            latest = timestamp2;
        if (timestamp3 < earliest)
            earliest = timestamp3;
        if (timestamp3 > latest)
            latest = timestamp3;
        if (timestamp4 < earliest)
            earliest = timestamp4;
        if (timestamp4 > latest)
            latest = timestamp4;
        if (timestamp5 < earliest)
            earliest = timestamp5;
        if (timestamp5 > latest)
            latest = timestamp5;
        if (timestamp6 < earliest)
            earliest = timestamp6;
        if (timestamp6 > latest)
            latest = timestamp6;
        if (latest - earliest > omax_time_disparity_)
            omax_time_disparity_ = latest - earliest;
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // cout << "sub1 " << fixed << setprecision(9) << ((rclcpp::Time)msg->timing.arrival_time).seconds() << endl;
        mdl_sync_.add<0>(msg);
        omdl_sync_.add<0>(msg);
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // cout << "sub2 " << fixed << setprecision(9) << ((rclcpp::Time)msg->timing.arrival_time).seconds() << endl;
        mdl_sync_.add<1>(msg);
        omdl_sync_.add<1>(msg);
    }
    
    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // cout << "sub3 " << fixed << setprecision(9) << ((rclcpp::Time)msg->timing.arrival_time).seconds() << endl;
        mdl_sync_.add<2>(msg);
        omdl_sync_.add<2>(msg);
    }
    
    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // cout << "sub3 " << fixed << setprecision(9) << ((rclcpp::Time)msg->timing.arrival_time).seconds() << endl;
        mdl_sync_.add<3>(msg);
        omdl_sync_.add<3>(msg);
    }
    
    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // cout << "sub3 " << fixed << setprecision(9) << ((rclcpp::Time)msg->timing.arrival_time).seconds() << endl;
        mdl_sync_.add<4>(msg);
        omdl_sync_.add<4>(msg);
    }
    
    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // cout << "sub3 " << fixed << setprecision(9) << ((rclcpp::Time)msg->timing.arrival_time).seconds() << endl;
        mdl_sync_.add<5>(msg);
        omdl_sync_.add<5>(msg);
    }


    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic5_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic6_sub_;

    typedef Synchronizer<sync_policies::LatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;
    typedef Synchronizer<sync_policies::OriginalLatestTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > OMdlSync;

    MdlSync mdl_sync_;
    OMdlSync omdl_sync_;

    int num_published_set_; // Required number of published sets
    
    int count_mdl_, count_omdl_;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., 
            max_reaction_latency4_ = 0., max_reaction_latency5_ = 0., max_reaction_latency6_ = 0.;
    double omax_reaction_latency1_ = 0., omax_reaction_latency2_ = 0., omax_reaction_latency3_ = 0., 
            omax_reaction_latency4_ = 0., omax_reaction_latency5_ = 0., omax_reaction_latency6_ = 0.;

    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_,
                previous_arrival_time4_, previous_arrival_time5_, previous_arrival_time6_;
    rclcpp::Time oprevious_arrival_time1_, oprevious_arrival_time2_, oprevious_arrival_time3_,
                oprevious_arrival_time4_, oprevious_arrival_time5_, oprevious_arrival_time6_;
    
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0.,
            max_passing_latency4_ = 0., max_passing_latency5_ = 0., max_passing_latency6_ = 0.;
    double omax_passing_latency1_ = 0., omax_passing_latency2_ = 0., omax_passing_latency3_ = 0.,
            omax_passing_latency4_ = 0., omax_passing_latency5_ = 0., omax_passing_latency6_ = 0.;
    
    double max_time_disparity_ = 0.;
    double omax_time_disparity_ = 0.;
    
};

int main(int argc, char * argv[])
{
    string result_path = argv[1];

    ofstream outfile;
    outfile.open(result_path, ios::app);
    if (!outfile.is_open())
    {
        cerr << "Error opening file " << result_path << endl;
        exit(-1);
    }
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);


    auto pub = std::make_shared<synchronizer::NPublisher>(6, 50);

    auto sub_node = std::make_shared<SubscriberTopic6>(outfile);

    executor.add_node(sub_node);

    executor.add_node(pub);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
