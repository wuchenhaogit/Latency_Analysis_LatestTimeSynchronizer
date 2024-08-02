#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h> // include the header file for sleep()

#include <string>
#include <vector>

#include "npublisher.h"
#include "latest_evaluation_subscriber.h"
#include "rclcpp/time_source.hpp"

using namespace std;

using namespace synchronizer;

int main(int argc, char * argv[])
{
    int channel_num = atoi(argv[1]);
    int lower_limit = atoi(argv[2]);
    string result_path = argv[3];
    int type, lower, upper;
    if (argc > 4) {
        type = atoi(argv[4]);
        lower = atoi(argv[5]); // will be divide by 10
        upper = atoi(argv[6]); // will be divide by 10
    }

    ofstream outfile_mdl;
    outfile_mdl.open(result_path + "/timestamp_mdl.txt", ios::app);
    if (!outfile_mdl.is_open())
    {
        cout<<"Error opening file timestamp_mdl.txt! "<<endl;
        exit(-1);
    }

    // set output accuracy
    outfile_mdl.setf(ios::fixed, ios::floatfield);
    outfile_mdl.precision(9);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);
    

    

    auto pub_node = std::make_shared<NPublisher>(channel_num, lower_limit);

    auto sub_node3 = std::make_shared<SubscriberTopic3>(outfile_mdl, type, lower, upper);
    auto sub_node4 = std::make_shared<SubscriberTopic4>(outfile_mdl, type, lower, upper);
    auto sub_node5 = std::make_shared<SubscriberTopic5>(outfile_mdl, type, lower, upper);
    auto sub_node6 = std::make_shared<SubscriberTopic6>(outfile_mdl, type, lower, upper);
    auto sub_node7 = std::make_shared<SubscriberTopic7>(outfile_mdl, type, lower, upper);
    auto sub_node8 = std::make_shared<SubscriberTopic8>(outfile_mdl, type, lower, upper);
    auto sub_node9 = std::make_shared<SubscriberTopic9>(outfile_mdl, type, lower, upper);
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

    executor.add_node(pub_node);

    executor.spin();


    rclcpp::shutdown();
    return 0;
}

