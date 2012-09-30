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

bool GotoThread::check_obstacles_in_path()
{
	int laser_obstacles = 0;
    for (size_t i=0; i<1080; i++)
    {
        if (laser_data[i] < robot_radius) laser_obstacles++;
    }

	if (laser_obstacles>=1) return true;
	return false;
}

void GotoThread::run()
{
	//data is formatted as follows: x, y, angle
    yarp::sig::Vector *loc = port_localization_input.read(false);
	if (loc) {localization_data = *loc; loc_timeout_counter=0;}
    else loc_timeout_counter++;

	yarp::sig::Vector *odm = port_odometry_input.read(false);
	if (odm) {odometry_data = *odm; odm_timeout_counter=0;}
    else odm_timeout_counter++;

    yarp::sig::Vector *las = port_laser_input.read(false);
    if (las) laser_data = *las;

	//computes the control action
	control_out.zero();

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
  control_out[0] =  180-(beta-localization_data[2]); //CHECKME -90
  

//printf ("%f \n", control[0]);
    control_out[1] =  k_lin_gain * distance;
    control_out[2] =  k_ang_gain * gamma;
    
    //saturation
    if (control_out[2] > +max_ang_speed) control_out[2] =  max_ang_speed;
    if (control_out[2] < -max_ang_speed) control_out[2] = -max_ang_speed;

    if (control_out[1] > +max_lin_speed) control_out[1] =  max_lin_speed;
    if (control_out[1] < -max_lin_speed) control_out[1] = -max_lin_speed;

    //check for obstacles
    if (enable_stop_on_obstacles)    
    {   
		bool obstacles_in_path = check_obstacles_in_path();
        if (status==MOVING && obstacles_in_path)
        {
            fprintf (stdout, "Obstacles detected, stopping \n");
            status=WAITING_OBSTACLE;
			yarp::os::Time::now();
        }
		else
		if (status==WAITING_OBSTACLE && !obstacles_in_path)
		{
            fprintf (stdout, "Obstacles removed, thank you \n");
            status=MOVING;
		}
    }

	//printf ("%f %f %f \n", gamma, beta, distance);
	if (status == MOVING)
	{
		if (fabs(distance) < 0.05 && fabs(gamma) < 0.6) 
		{
			status=REACHED;
            fprintf (stdout, "Goal reached!\n");
		}
	}

	if (status==WAITING_OBSTACLE)
	{
		double current_time = yarp::os::Time::now();
		if (fabs(current_time-obstacle_time)>30.0)
		{
			fprintf (stdout, "failed to recover from obstacle, goal aborted \n");
			status=ABORTED;
		}
	}

	if (status==PAUSED)
	{
		//check if apuse is expired
		double current_time = yarp::os::Time::now();
		if (current_time - pause_start > pause_duration)
		{
			fprintf(stdout, "pause expired! resuming \n");
			status=MOVING;
		}
	}

    if (status != MOVING)
    {
       control_out[0]=control_out[1]=control_out[2] = 0.0;        
    }

    if (enable_retreat && retreat_counter >0)
    {
        control_out[0]=180;
        control_out[1]=0.4;
        control_out[2]=0;
        retreat_counter--;
    }

	//update status string
	status_string = "ERROR";
	switch (status)
	{
		case IDLE:
		status_string = "IDLE";
		break;
		case MOVING:
		status_string = "MOVING";
		break;
		case WAITING_OBSTACLE:
		status_string = "WAITING_OBSTACLE";
		break;
		case REACHED:	
		status_string = "REACHED";
		break;
		case ABORTED:
		status_string = "ABORTED";
		break;
		case PAUSED:
		status_string = "PAUSED";
		break;
	}

	sendOutput();
}

void GotoThread::sendOutput()
{
	//send the motors commands and the status to the yarp ports
	if (port_commands_output.getOutputCount()>0)
	{
		Bottle &b=port_commands_output.prepare();
		b.clear();
		b.addInt(2);                    // polar commands
		b.addDouble(control_out[0]);    // angle in deg
		b.addDouble(control_out[1]);    // lin_vel in m/s
		b.addDouble(control_out[2]);    // ang_vel in deg/s
		port_commands_output.write();
	}

	if (port_status_output.getOutputCount()>0)
	{
		string string_out;
		string_out = status_string;
		Bottle &b=port_status_output.prepare();
		b.clear();
		b.addString(string_out.c_str());
		port_status_output.write();
	}
}

void GotoThread::setNewAbsTarget(yarp::sig::Vector target)
{
	//data is formatted as follows: x, y, angle
	target_data=target;
    status=MOVING;
	fprintf (stdout, "received new target\n");
    retreat_counter = retreat_duration;
}

void GotoThread::setNewRelTarget(yarp::sig::Vector target)
{
	//data is formatted as follows: x, y, angle
	if (target.size()==2) target.push_back(0.0);
	double a = localization_data[2]/180.0*M_PI;
	target_data[0]=target[1] * cos (a) - (-target[0]) * sin (a) + localization_data[0] ;
	target_data[1]=target[1] * sin (a) + (-target[0]) * cos (a) + localization_data[1] ;
	target_data[2]=-target[2] + localization_data[2];
    status=MOVING;
	fprintf (stdout, "received new target\n");
    retreat_counter = retreat_duration;
}

void GotoThread::pauseMovement(double secs)
{
	if (status == PAUSED)
	{
		fprintf (stdout, "already in pause!\n");
		return;
	}
	if (status != MOVING)
	{
		fprintf (stdout, "not moving!\n");
		return;
	}

	if (secs > 0)
	{
		fprintf (stdout, "asked to pause for %f \n", secs);
		pause_duration = secs;
	}
	else
	{
		fprintf (stdout, "asked to pause\n");
		pause_duration = 10000000;
	}
    status=PAUSED;
	pause_start = yarp::os::Time::now();
}

void GotoThread::resumeMovement()
{
	fprintf (stdout, "asked to resume movement\n");
    status=MOVING;
}

void GotoThread::stopMovement()
{
	fprintf (stdout, "asked to stop\n");
    status=IDLE;
}

string GotoThread::getNavigationStatus()
{
	return status_string;
}

void GotoThread::printStats()
{
	fprintf (stdout,"\n");
    fprintf (stdout,"* ikartGoto thread:\n");
    fprintf (stdout,"loc timeouts: %d\n",loc_timeout_counter);
    fprintf (stdout,"odm timeouts: %d\n",odm_timeout_counter);
	fprintf (stdout,"status: %s\n",status_string.c_str());
}
