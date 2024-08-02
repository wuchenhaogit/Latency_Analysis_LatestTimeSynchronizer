#!/bin/bash

source ../../install/setup.bash

# Predefined Parameter
num_group=5

for (( k=0; k<num_group; k++))
do  
    ##############################################
    # Evaluation with varied timestamp separation 
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=0; j<5; j++ ))
    do
        echo "-------------------Evaluation for varied timestamp separation: $j, group: $k.-------------------"
        factor=$((2 * j))

        # Launch each executable in a loop and save the PID to array
        config_index=$((2 + j))
        
        ros2 run latency_analysis latest_model_test $channel_num $period_lower_limit --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml


        # Wait for all executables to complete
        wait
    done

    ##############################################
    # Evaluation with different number of channels
    ##############################################
    period_lower_limit=50
    for (( j=3; j<10; j++ ))
    do
        echo "-------------------Evaluation for different number of channels: $j, group: $k.-------------------"
        channel_num=$j


        ros2 run latency_analysis latest_model_test $channel_num $period_lower_limit --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_1.yaml

        # Sleep for 2 seconds

        # Wait for all executables to complete
        wait
    done
    
    ##############################################
    # Evaluation with varied delay
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=1; j<6; j++ ))
    do
        echo "-------------------Evaluation for varied delay: $j, group: $k.-------------------"
        delay_upper=$((10 * j))

        # Launch each executable in a loop and save the PID to array
        config_index=$((6 + j))
        
        ros2 run latency_analysis latest_model_test $channel_num $period_lower_limit --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml


        # Wait for all executables to complete
        wait
    done

done
wait

exit



