#!/bin/bash

source ../../install/setup.bash

# Predefined Parameter
num_group=5


for (( k=0; k<num_group; k++))
do  
    ##############################################
    # Evaluation with varied period factor 
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=0; j<7; j++ ))
    do
        echo "-------------------Evaluation for varied period factor: $j, group: $k.-------------------"
        factor=$((2 * j))
        result_path="./results/evaluation/varied_period_factor/varied_period_factor_1.$factor"

        config_index=$((2 + j))

        if [ $j -eq 5 ]
        then
            result_path="./results/evaluation/varied_period_factor/varied_period_factor_4.0"
            config_index=13
        elif [ $j -eq 6 ]
        then
            result_path="./results/evaluation/varied_period_factor/varied_period_factor_8.0"
            config_index=14
        fi

        ros2 run latency_analysis latest_evaluation $channel_num $period_lower_limit $result_path --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml

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
        result_path="./results/evaluation/channel_num/channel_$channel_num"

        ros2 run latency_analysis latest_evaluation $channel_num $period_lower_limit $result_path --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_1.yaml

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
        result_path="./results/evaluation/varied_delay/random_delay_$delay_upper"

        if [ $j -eq 5 ]
        then
            result_path="./results/evaluation/varied_delay/random_nodelay"
        fi
        
        config_index=$((6 + j))
        ros2 run latency_analysis latest_evaluation $channel_num $period_lower_limit $result_path --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml

        # Wait for all executables to complete
        wait
    done

    ##############################################
    # Evaluation with varied EMA weight of frequency
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=0; j<7; j++ ))
    do
        echo "-------------------Evaluation for varied EMA frequency: $j, group: $k.-------------------"
        lower=$((2*j - 2))
        upper=$((2*j))

        if [ $j -eq 0 ]
        then
            lower=0
        elif [ $j -eq 6 ]
        then
            upper=10
        fi


        result_path="./results/evaluation/varied_EMA_frequency_weight/EMA_frequency_weight_$j"

        config_index=4

        ros2 run latency_analysis latest_evaluation $channel_num $period_lower_limit $result_path 1 $lower $upper --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml


        # Wait for all executables to complete
        wait
    done

    ##############################################
    # Evaluation with varied EMA weight of error
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=0; j<7; j++ ))
    do
        echo "-------------------Evaluation for varied EMA error: $j, group: $k.-------------------"
        lower=$((2*j - 2))
        upper=$((2*j))

        if [ $j -eq 0 ]
        then
            lower=0
        elif [ $j -eq 6 ]
        then
            upper=10
        fi


        result_path="./results/evaluation/varied_EMA_error_weight/EMA_error_weight_$j"

        config_index=4

        ros2 run latency_analysis latest_evaluation $channel_num $period_lower_limit $result_path 2 $lower $upper --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml


        # Wait for all executables to complete
        wait
    done

    ##############################################
    # Evaluation with varied margin factor
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=0; j<7; j++ ))
    do
        echo "-------------------Evaluation for margin factor: $j, group: $k.-------------------"
        lower=$((2**j / 2))
        upper=$((2**j))

        if [ $j -eq 0 ]
        then
            lower=0
            upper=0
        elif [ $j -eq 1 ]
        then
            lower=0
        fi


        result_path="./results/evaluation/varied_margin_factor/margin_factor_$j"

        config_index=4

        ros2 run latency_analysis latest_evaluation $channel_num $period_lower_limit $result_path 3 $lower $upper --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml


        # Wait for all executables to complete
        wait
    done

done
wait

exit

