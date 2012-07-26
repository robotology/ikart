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
	//data is formatted as follows: x, y, angle
    yarp::sig::Vector *loc = port_localization_input.read(false);
    if (loc) localization_data = *loc;
    else timeout_counter++;

    yarp::sig::Vector *las = port_laser_input.read(false);
    if (las) laser_data = *las;

	//computes the control action
	yarp::sig::Vector control;
	control.resize(3,0.0);

	//gamma is the angle between the current ikart heading and the target heading
	double gamma = localization_data[2]-target_data[2];

	//beta is the angle between the current ikart position and the target position
	double beta = atan2 (localization_data[1]-target_data[1],localization_data[0]-target_data[0])*180.0/M_PI;

	//distance is the distance between the current ikart position and the target position
    double distance = sqrt(pow(target_data[0]-localization_data[0],2) +  pow(target_data[1]-localization_data[1],2));

    //compute the control law
//    control[0] = -beta;
    //control[0] = -beta-localization_data[2]; //CHECKME
//   control[0] = -(beta-localization_data[2]); //CHECKME -180
//   control[0] = -(beta+localization_data[2]); //CHECKME -90
 //   control[0] = +beta+localization_data[2]-90; //CHECKME -90
 //   control[0] = +beta-localization_data[2]-90; //CHECKME -90
 //   control[0] = -beta+localization_data[2]-90; //CHECKME -90
//   control[0] = -beta-localization_data[2]-90; //CHECKME -90
 //  control[0] = -beta-localization_data[2]+90; //CHECKME -90
  // control[0] = -beta+localization_data[2]+90; //CHECKME -90
 //  control[0] = +beta+localization_data[2]+90; //CHECKME -90
 //  control[0] = +beta-localization_data[2]+90; //CHECKME -90
   //  control[0] = -beta+localization_data[2]; //CHECKME -90
   //  control[0] =  beta-localization_data[2]; //CHECKME -90
  control[0] =  180-(beta-localization_data[2]); //CHECKME -90
  

//printf ("%f \n", control[0]);
    control[1] =  k_lin_gain * distance;
    control[2] =  k_ang_gain * gamma;
    
    //saturation
    if (control[2] > +max_ang_speed) control[2] =  max_ang_speed;
    if (control[2] < -max_ang_speed) control[2] = -max_ang_speed;

    if (control[1] > +max_lin_speed) control[1] =  max_lin_speed;
    if (control[1] < -max_lin_speed) control[1] = -max_lin_speed;

    //check for obstacles
    if (enable_stop_on_obstacles)    
    {   int laser_obstacles = 0;
        if (las)    
        {   
            for (size_t i=0; i<1080; i++)
            {
                if ((*las)[i] < robot_radius) laser_obstacles++;
            }
        }
        if (laser_obstacles)
        {
            fprintf (stdout, "Obstacles detected, stopping /n");
            status="ABORTED";
        }
    }

	//printf ("%f %f %f \n", gamma, beta, distance);
	/*if (status == "rotate")
	{
		if (fabs(distance) < 0.05 && fabs(gamma) < 0.6) 
		{
            fprintf (stdout, "Goal reached! /n");
		}
	}*/

    if (status != "MOVING")
    {
       control[0]=control[1]=control[2] = 0.0;        
    }

    if (enable_retreat && retreat_counter >0)
    {
        control[0]=180;
        control[1]=0.4;
        control[2]=0;
        retreat_counter--;
    }

    Bottle &b=port_commands_output.prepare();
    b.clear();
    b.addInt(2);                // polar commands
    b.addDouble(control[0]);    // angle in deg
    b.addDouble(control[1]);    // lin_vel in m/s
    b.addDouble(control[2]);    // ang_vel in deg/s
    port_commands_output.write();
}

void GotoThread::setNewTarget(yarp::sig::Vector target)
{
	//data is formatted as follows: x, y, angle
	target_data=target;
    status="MOVING";
    retreat_counter = retreat_duration;
}

void GotoThread::stopMovement()
{
    status="IDLE";
}

void GotoThread::printStats()
{
    fprintf (stdout,"* ikartGoto thread:\n");
    fprintf (stdout,"timeouts: %d\n",timeout_counter);
}
