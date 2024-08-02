#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h> // include the header file for sleep()

#include <string>
#include <vector>

#include "publisher.h"
#include "model_evaluation_subscriber.h"
#include "rclcpp/time_source.hpp"

using namespace std;

using namespace synchronizer;

int main(int argc, char * argv[])
{
    int channel_num = atoi(argv[1]);
    int lower_limit = atoi(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);
    
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);
    uniform_int_distribution<unsigned> perd(lower_limit, 100);
    
    std::vector<std::shared_ptr<Publisher>> pub_nodes;

    for (int i = 0 ; i<channel_num ; ++i) {
        int real_period = perd(gen);
        pub_nodes.push_back(std::make_shared<Publisher>("topic"+to_string(i+1), real_period));
    }

    rclcpp::TimeSource time_source;
    auto sync_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    for (int i = 0 ; i<channel_num ; ++i)
        time_source.attachNode(pub_nodes[i]);
    time_source.attachClock(sync_clock);

    auto sub_node3 = std::make_shared<SubscriberTopic3>(sync_clock);
    auto sub_node4 = std::make_shared<SubscriberTopic4>(sync_clock);
    auto sub_node5 = std::make_shared<SubscriberTopic5>(sync_clock);
    auto sub_node6 = std::make_shared<SubscriberTopic6>(sync_clock);
    auto sub_node7 = std::make_shared<SubscriberTopic7>(sync_clock);
    auto sub_node8 = std::make_shared<SubscriberTopic8>(sync_clock);
    auto sub_node9 = std::make_shared<SubscriberTopic9>(sync_clock);
    switch (channel_num) {
        case 3:
            executor.add_node(sub_node3);
            break;
        case 4:
            executor.add_node(sub_node4);
            break;
        case 5:
            executor.add_node(sub_node5);
            break;
        case 6:
            executor.add_node(sub_node6);
            break;
        case 7:
            executor.add_node(sub_node7);
            break;
        case 8:
            executor.add_node(sub_node8);
            break;
        case 9:
            executor.add_node(sub_node9);
            break;
    }

    for (int i = 0 ; i<channel_num ; ++i)
        executor.add_node(pub_nodes[i]);

    executor.spin();


    rclcpp::shutdown();
    return 0;
}

