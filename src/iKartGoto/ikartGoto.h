/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef COMPASS_THREAD_H
#define COMPASS_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class GotoThread: public yarp::os::RateThread
{
    protected:
    //configuration parameters
    double k_ang_gain;
    double k_lin_gain;
    double max_lin_speed;    //m/s
    double max_ang_speed;    //deg/s
    double robot_radius;     //m
    int    retreat_duration; 
    bool   enable_stop_on_obstacles;
    bool   enable_retreat;

    //ports
	BufferedPort<yarp::sig::Vector> port_localization_input;
	BufferedPort<yarp::sig::Vector> port_target_input;
    BufferedPort<yarp::sig::Vector> port_laser_input;
    BufferedPort<yarp::os::Bottle>  port_commands_output;

    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   target_data;
    yarp::sig::Vector   laser_data;
	string              status;
    int                 timeout_counter;
    int                 retreat_counter;

    public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        status = "IDLE";
        timeout_counter     = 0;
        localization_data.resize(3,0.0);
        target_data.resize(3,0.0);
        laser_data.resize(1080,100.0);
        retreat_counter = 0;
    }

    virtual bool threadInit()
    {
        //read configuration parametes
        k_ang_gain = 0.05;
        k_lin_gain = 0.1;
        max_lin_speed = 0.9;  //m/s
        max_ang_speed = 10.0; //deg/s
        robot_radius = 0.30;  //m
        enable_stop_on_obstacles = false;
        enable_retreat = false;
        retreat_duration = 300;

        //open module ports
		string localName = "/ikart/goto";
        port_localization_input.open((localName+"/localization:i").c_str());
        port_target_input.open((localName+"/target:i").c_str());
		port_laser_input.open((localName+"/laser:i").c_str());
		port_commands_output.open((localName+"/commands:o").c_str());

        //automatic port connections
        bool b = false;
        b = Network::connect("/ikart_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the localization port!"); return false;}
        b = Network::connect((localName+"/commands:o").c_str(),"/ikart/control:i", "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the output command port!"); return false;}
        b = Network::connect("/ikart/laser:o",(localName+"/laser:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the laser port!"); }
        return true;
    }

    virtual void run();

	void setNewTarget(yarp::sig::Vector target);
	void stopMovement();

    virtual void threadRelease()
    {    
        port_localization_input.interrupt();
        port_localization_input.close();
        port_target_input.interrupt();
        port_target_input.close();
		port_laser_input.interrupt();
        port_laser_input.close();
	    port_commands_output.interrupt();
        port_commands_output.close();
    }

    void printStats();

};

#endif
