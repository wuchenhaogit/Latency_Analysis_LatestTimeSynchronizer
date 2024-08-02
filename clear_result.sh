for (( j=3; j<10; j++ ))
do
    result_path="./results/evaluation/channel_num/channel_$j"
    rm $result_path/timestamp_mdl.txt
    touch $result_path/timestamp_mdl.txt
done

for (( j=1; j<6; j++ ))
do
    result_path="./results/evaluation/varied_delay/random_delay_$((10 * j))"
    if [ $j -eq 5 ]
    then
        result_path="./results/evaluation/varied_delay/random_nodelay"
    fi
    rm $result_path/timestamp_mdl.txt
    touch $result_path/timestamp_mdl.txt
done

for (( j=0; j<5; j++ ))
do
    result_path="./results/evaluation/varied_period_factor/varied_period_factor_1.$((2 * j))"
    rm $result_path/timestamp_mdl.txt
    touch $result_path/timestamp_mdl.txt
done

for (( j=0; j<7; j++ ))
do
    result_path="./results/evaluation/varied_EMA_frequency_weight/EMA_frequency_weight_$j"
    rm $result_path/timestamp_mdl.txt
    touch $result_path/timestamp_mdl.txt
    result_path="./results/evaluation/varied_EMA_error_weight/EMA_error_weight_$j"
    rm $result_path/timestamp_mdl.txt
    touch $result_path/timestamp_mdl.txt
    result_path="./results/evaluation/varied_margin_factor/margin_factor_$j"
    rm $result_path/timestamp_mdl.txt
    touch $result_path/timestamp_mdl.txt
done


