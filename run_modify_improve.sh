#!/bin/bash

source ../../install/setup.bash

# Predefined Parameter
num_group=20


for (( k=1; k<=num_group; k++))
do  
    ##############################################
    # Evaluation with varied period factor 
    ##############################################
    for (( j=0; j<7; j++ ))
    do
        echo "-------------------Evaluation for varied period factor: $j, group: $k.-------------------"
        factor=$((2 * j))
        result_path="./results/revision/varied_period_factor/varied_period_factor_1.$factor"

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

        ros2 run latency_analysis modify_improve_test $result_path/timestamp_mdl.txt  --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml

        # Wait for all executables to complete
        wait
    done
    
    ##############################################
    # Evaluation with varied delay
    ##############################################
    for (( j=1; j<6; j++ ))
    do
        echo "-------------------Evaluation for varied delay: $j, group: $k.-------------------"
        delay_upper=$((10 * j))
        result_path="./results/revision/varied_delay/random_delay_$delay_upper"

        if [ $j -eq 5 ]
        then
            result_path="./results/revision/varied_delay/random_nodelay"
        fi

        ros2 run latency_analysis modify_improve_test $result_path/timestamp_mdl.txt --ros-args --params-file ~/ros2_iron/src/latency_analysis/config/config_$config_index.yaml

        # Wait for all executables to complete
        wait
    done

done
wait

exit

