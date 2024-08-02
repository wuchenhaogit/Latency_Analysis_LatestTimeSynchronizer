# latency_analysis

## Modelling and Analysis of the LatestTime Message Synchronization Policy in ROS

This is a **ROS2 Iron** package providing the evaluation method for the analysis of the LatestTime message synchronizer in ROS [LatestTime policy](https://github.com/ros2/message_filters/blob/master/include/message_filters/sync_policies/latest_time.h).

## Package Structure
```
> tree .
|-- config
    |-- config_x.yaml                         # ROS2 parameters, i.e., test setting
|-- include         
    |-- helper.h                              # Some helper functions    
    |-- publisher.h                           # Message publishing node using timer. There is a delay time before publishing
    |-- npublisher.h                          # Message multiple type of publisher.h
    |-- model_evaluation_subscriber.h         # Message subscription node for comparison of our model (`OriginalLatestTimeModel`) and the original policy.
    |-- latest_evaluation_subscriber.h        # Message subscription node for evaluation of timing metrics of our revised model (`LatestTimeModel`).
|-- message_filters                           # Files in this directory should be put into `message_filters` package
    |-- sync_policies
        |-- original_latest_time_model.h      # Our implementation for modeling LatestTime policy
        |-- latest_time_model.h               # Our model after revision
        |-- latest_time.h                     # Modified latest_time.h. Use manually generated arriving time to control the delay.
|-- msg
    |-- Timing.msg                            # Timing data structure, i.e., ROS 2 interface, for message timing trace
|-- src
    |-- latest_model_test.cpp                 # main function for comparing our model and original policy.
    |-- modify_improve_test.cpp               # code for evaluating our revision.
    |-- latest_evaluation.cpp                 # main function for evaluating the our bounds using revised model.
|-- latest_model_test.sh                      # The script for running model validation experiment.
|-- run_modify_improve.sh                     # The script for running revision evaluation experiment.
|-- run_latest_evaluation.sh                  # The script for running bound evaluation experiment.
|-- clear_result.sh                           # The script to reset the result.
```

## Before building

### Environment
- [Build ROS2 Iron in `ros2_iron` workspace](https://docs.ros.org/en/iron/Installation/Ubuntu-Development-Setup.html)
- Clone this project into your workspace (under 'src' directory)

### Files preparation

- Copy and replace files in `message_filters/sync_policies` to `ros2_iron/src/ros2/message_filters/include/message_filters/sync_policies`
- Copy file `msg/Timing.msg` to `/ros2_iron/src/ros2/common_interfaces/std_msgs/msg/`
- Add the following line to file `/ros2_iron/src/ros2/common_interfaces/std_msgs/CMakeLists.txt`, just after `set(msg_files`
    ```sh
    "msg/Timing.msg"
    ```
- Modify content of file `/ros2_iron/src/ros2/common_interfaces/sensor_msgs/msg/JointState.msg` as follows:
    ```sh
    std_msgs/Timing timing   # This line should be added
    std_msgs/Header header
    ```

## Build

### Build interface
 ```sh
cd ros2_iron
source ./install/setup.bash
colcon build --packages-select std_msgs --symlink-install
source ./install/setup.bash
colcon build --packages-select sensor_msgs --symlink-install
 ```

### Build `message_filters` package

 ```sh
cd ros2_iron
source ./install/setup.bash
colcon build --packages-select message_filters --symlink-install
 ```

### Build this project
```sh
cd ros2_iron
source ./install/setup.bash
colcon build --packages-select latency_analysis --symlink-install
```

## Run evaluation

```sh
cd ~/ros2_iron/src/latency_analysis
chmod +x run_*.sh
./run_*.sh
```

**Note**: You can modify the parameter `num_group` in line 6 of `run_*.sh` to set the number of groups to be tested. The default value is set as 5 (which is 50 for the representive result in our paper) for shorter running times. Also for the same reason, the configuration parameter `num_published_set` (i.e., the number of published sets for each group) in configuration files is set as 2000, the same as our paper.
