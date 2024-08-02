/*
Delay operation executes here.
*/
#include <chrono>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include "helper.h"

#include <random>
using namespace std;

using namespace std::chrono_literals;

namespace synchronizer
{

class NPublisher : public rclcpp::Node
{
public:
  NPublisher(int channel_num, int lower_limit)
  : Node("N"), channel_num_(channel_num), thread_created_(false)
  {
    for (int i = 0 ; i<9 ; ++i)
      publisher_[i] = this->create_publisher<sensor_msgs::msg::JointState>(std::string("topic")+to_string(i+1), 10);

    this->declare_parameter("msg_delay_opt", 0);
    this->declare_parameter("delay_upper", 0);
    // this->declare_parameter("sample_upper", 50);

    this->get_parameter("msg_delay_opt", msg_delay_opt_);
    this->get_parameter("delay_upper", delay_upper_);
    // this->get_parameter("sample_upper", sample_upper_);


    // RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d.", msg_delay_opt_, delay_upper_);

    this->declare_parameter("period_factor", 1.0);
    this->declare_parameter("msg_period_bias_flag", false);

    this->get_parameter("period_factor", period_factor_);
    this->get_parameter("msg_period_bias_flag", msg_period_bias_flag_);

    if(msg_period_bias_flag_)
    {
      // RCLCPP_INFO(this->get_logger(), "Period bias has been set as: %.1f", period_factor_);
    }

    // RCLCPP_INFO(this->get_logger(), "Period of %s: %d", topic_name.c_str(), real_period_);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);
    uniform_int_distribution<unsigned> perd(lower_limit, 100);

    for(int i = 0 ; i<channel_num ; ++i){
      real_period_[i] = perd(gen);
      period_bias_upper_[i] = (period_factor_ - 1) * real_period_[i];
    }

    Thread = new std::thread(&NPublisher::timer_callback,this);
  }

private:
  void thread_create()
  {
    if(!thread_created_)
    {
      thread_created_ = true;
      Thread = new std::thread(&NPublisher::timer_callback,this);
      // Thread->join();
    }
  }

  void timer_callback()
  { 
    rclcpp::Time start = rclcpp::Time(0,0);
    sensor_msgs::msg::JointState messages[9];
    for (int i = 0 ; i<9 ; ++i) {
      messages[i].header.stamp = start;
      messages[i].timing.timestamp = start;
      messages[i].timing.arrival_time = start;
    }
    rclcpp::Time prev = rclcpp::Time(0,0);

    while(rclcpp::ok())
    {
      int argmin = 0;
      for (int i = 1 ; i<channel_num_ ; ++i)
        if ((rclcpp::Time)messages[i].timing.arrival_time < (rclcpp::Time)messages[argmin].timing.arrival_time)
          argmin = i;
      
      rclcpp::sleep_for(std::chrono::milliseconds((int)((((rclcpp::Time)messages[argmin].timing.arrival_time).seconds() - prev.seconds())*1000)));
      // rclcpp::sleep_for(std::chrono::milliseconds(20));
      publisher_[argmin]->publish(messages[argmin]);
      prev = (rclcpp::Time)messages[argmin].timing.arrival_time;
      

      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      default_random_engine e(seed);
      uniform_int_distribution<unsigned> bias(0, period_bias_upper_[argmin]); // bias for T
      uniform_int_distribution<unsigned> bias_flag(0, 1); // bias or not

      int period_bias = 0;
      if(msg_period_bias_flag_)
      {
        int flag = bias_flag(e);
        int b = bias(e);
        period_bias = flag * b;
      }

      if(msg_delay_opt_ == 1)
      {
          delay_some_time(delay_previous_[argmin], period_previous_[argmin]); // delay 1-40ms always
      }
      else if(msg_delay_opt_ == 2)
      {
          delay_random_time(delay_previous_[argmin], period_previous_[argmin], delay_upper_); // delay 0-upper ms
      }
      period_previous_[argmin] = real_period_[argmin]+period_bias;

      
      rclcpp::Time last_timestamp = (rclcpp::Time)messages[argmin].header.stamp;
      messages[argmin].header.stamp = last_timestamp + rclcpp::Duration(0, (real_period_[argmin]+period_bias)*1e6);
      messages[argmin].timing.timestamp = last_timestamp + rclcpp::Duration(0, (real_period_[argmin]+period_bias)*1e6);
      messages[argmin].timing.arrival_time = last_timestamp + rclcpp::Duration(0, (real_period_[argmin]+period_bias+delay_previous_[argmin])*1e6);
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_[9];

  bool thread_created_;

  int real_period_[9];

  double period_factor_;
  bool msg_period_bias_flag_;
  double period_bias_upper_[9];
  int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
  int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
  double delay_previous_[9] = {0.};
  int period_previous_[9] = {0};
  int channel_num_ = 6;

  std::thread* Thread;
};

}


