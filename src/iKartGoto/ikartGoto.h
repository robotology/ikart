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
	BufferedPort<yarp::sig::Vector> port_localization_input;
	BufferedPort<yarp::sig::Vector> port_target_input;
    BufferedPort<yarp::sig::Vector> port_laser_input;
    BufferedPort<yarp::os::Bottle>  port_commands_output;

    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   target_data;
	string              status;
    int                 timeout_counter;

    public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        timeout_counter     = 0;
        localization_data.resize(3,0.0);
        target_data.resize(3,0.0);
    }

    virtual bool threadInit()
    {
		string localName = "/ikart/goto";
        port_localization_input.open((localName+"/localization:i").c_str());
        port_target_input.open((localName+"/target:i").c_str());
		port_laser_input.open((localName+"/laser:i").c_str());
		port_commands_output.open((localName+"/commands:o").c_str());
        //Network::connect("/icub/inertial",(localName+"/inertial:i").c_str());
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