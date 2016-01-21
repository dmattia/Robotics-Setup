#! /bin/bash

line=$(ifconfig wlan0 | grep 'inet addr')
re="inet addr:([0-9\.]*)"
if [[ $line =~ $re ]];
then
	ip_addr=${BASH_REMATCH[1]};
else
	echo "Could not find ip address"
	echo "Please check network connection and try again"
	exit 1
fi

export ROS_HOSTNAME=$ip_addr
export ROS_MASTER_URI="http://$ip_addr:11311"

source ~/.bashrc
echo "Success! ip address is $ip_addr"
