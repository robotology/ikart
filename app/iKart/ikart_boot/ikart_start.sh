#!/bin/sh

sudo -u icub -i joystickCheck
ret1=$?

if [ $ret1 -eq 101 ]; then
    echo "joystick boot requested" 
    sudo -u icub -i yarp server &
    #sudo -u icub -i icub-cluster-server.sh start yarpserver3 &
    sleep 2
    sudo -u icub -i joystickCtrl --context joystickCtrl --from ikart.ini --silent --force_configuration &
    sudo -u icub -i yarprobotinterface --context iKart --config ikart_main.xml &
    sudo -u icub -i baseControl --context navigation --from baseCtrl_ikartV1.ini --skip_robot_interface_check --joystick_connect &
elif [ $ret1 -eq 0 ]; then
    echo "No joystick boot requested" 
else
    echo "Unknown error" 
fi
