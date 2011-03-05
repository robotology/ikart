/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#define PRINT_STATUS_PER    0.5     // [s]

class CompassThread: public yarp::os::RateThread
{
	protected:
	Property iKartCtrl_options;

	ResourceFinder      &rf;
	BufferedPort<yarp::sig::Vector> port_inertial_input;
	BufferedPort<yarp::sig::Vector> port_compass_output;
	yarp::sig::Vector compass_data;
	yarp::sig::Vector inertial_data;
	string remoteName;
    string localName;

	public:
    
	CompassThread(unsigned int _period, ResourceFinder &_rf, Property options,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
			   iKartCtrl_options (options),
               remoteName(_remoteName), localName(_localName) 
	{
		inertial_data.resize(12,0.0);
		compass_data.resize(3,0.0);	
	}

    virtual bool threadInit()
    {
		port_inertial_input.open((localName+"/inertial:i").c_str());
		port_compass_output.open((localName+"/compass:o").c_str());
		
		return true;
	}

	virtual void run()
	{		
		yarp::sig::Vector *iner = port_inertial_input.read(false);
		if (iner) inertial_data = *iner;

		//add here kinematics computation
		compass_data[0]=inertial_data[5];
		compass_data[1]=inertial_data[4];
		compass_data[2]=inertial_data[3];

		yarp::sig::Vector &pcompass_data=port_compass_output.prepare();
		pcompass_data=compass_data;
		//lastStateStamp.update();
		//port_compass_data.setEnvelope(lastStateStamp);
		port_compass_output.write();
	}

	virtual void threadRelease()
    {    
		port_inertial_input.interrupt();
        port_inertial_input.close();
		port_compass_output.interrupt();
        port_compass_output.close();
    }
};

#endif