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
#include <math.h>

#include "ikartGoto.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

void GotoThread::run()
{
    yarp::sig::Vector *loc = port_localization_input.read(false);
    if (loc) localization_data = *loc;
    else timeout_counter++;
	//data is formatted as follows: x, y, angle

	//computes the control action
	yarp::sig::Vector control;
	control.resize(3,0.0);

    //add here kinematics computation
    control[0]=localization_data[2];
    control[1]=localization_data[1];
    control[2]=localization_data[0];

	//gamma is the angle between the current ikart heading and the target heading
	double gamma = localization_data[2]-target_data[2];

	//beta is the angle between the current ikart heading and the target position
	double beta = atan2 (localization_data[1]-target_data[1],localization_data[0]-target_data[0])*180.0/M_PI;

	//distance is the distance between the current ikart position and the target position
	double distance = sqrt(localization_data[0]*localization_data[0] + localization_data[1]*localization_data[1])-
                      sqrt(target_data[0]*target_data[0] + target_data[1]*target_data[1]);

	printf ("%f %f %f \n", gamma, beta, distance);
	if (status == "rotate")
	{
		if (localization_data[2] - beta > 1) 
		{
		}
		else if (localization_data[2] - beta < 1) 
		{
		}
		else
		{

		}
	}

    Bottle &b=port_commands_output.prepare();
    b.clear();
    b.addInt(3);
    b.addDouble(control[0]);    // x_vel in m/s
    b.addDouble(control[1]);    // y_vel in m/s
    b.addDouble(control[2]);    // t_vel in deg/s
    port_commands_output.write();
}

void GotoThread::setNewTarget(yarp::sig::Vector target)
{
	//data is formatted as follows: x, y, angle
	target_data=target;
}

void GotoThread::stopMovement()
{
}

void GotoThread::printStats()
{
    fprintf (stdout,"* ikartGoto thread:\n");
    fprintf (stdout,"timeouts: %d\n",timeout_counter);
}
