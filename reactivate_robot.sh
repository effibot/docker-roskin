#! /bin/sh

# this script will connect to the robot through ssh and reactivate it

# get the robot's IP address from the user
echo "Please enter the robot's IP address: "
read ip_address
# store the IP address in a variable
ip=$ip_address
# connect to the robot through ssh
ssh -N nao@$ip
# deactivate the robot
sh nao stop && sh naoqi-bin --disable-life >> /dev/null &
# wait for 5 seconds
sleep 5
# reconnect to the robot through ssh
ssh -N nao@$ip
# reactivate the robot
sh qicli call ALMotion.wakeUp
exit 0
