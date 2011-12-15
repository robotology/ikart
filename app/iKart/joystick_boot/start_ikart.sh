#!/bin/sh

sudo -u icub -i joystickCheck
ret1=$?

if [ $ret1 -eq 101 ]; then
    echo "joystick boot requested" 
    sudo -u icub -i yarp server &
    sleep 2
    sudo -u icub -i iCubInterface --context iKart --config conf/iKart.ini &
    sleep 2
    sudo -u icub -i iKartCtrl --no_compass &
    sleep 2
    sudo -u icub -i joystickCtrl --context joystickCtrl --from conf/xbox_linux.ini --silent &
    sleep 2
    sudo -u icub -i yarp connect /joystickCtrl:o /ikart/joystick:i
elif [ $ret1 -eq 0 ]; then
    echo "error 0" 
elif [ $ret1 -eq -1 ]; then
    echo "error -1" 
elif [ $ret1 -eq -2 ]; then
    echo "error -2" 
else
    echo "No joystick boot requested" 
fi
