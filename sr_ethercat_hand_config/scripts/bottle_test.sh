#!/bin/bash

bag_path=~/Documents/optoforce_data
mkdir -p $bag_path
time_now="$(date +"%I%M%S")"
bag_name=optoforce_bag_$time_now
cd $bag_path

rosbag record -O $bag_name.bag -e "/optoforce(.*)|/joint_states" &
bag_PID=$!

rosrun sr_ethercat_hand_config bottle_squeeze_example.py

bag_PIDs=$(pgrep -P $bag_PID)
kill -2 $bag_PIDs

csv_path=~/Documents/optoforce_data/$bag_name"_csv"
mkdir -p $csv_path

wait

for topic in `rostopic list -b $bag_name.bag` ; do rostopic echo -p -b $bag_name.bag $topic > $csv_path/$bag_name${topic//\//_}.csv ; done
