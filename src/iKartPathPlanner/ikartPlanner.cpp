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
#include <cv.h>
#include <highgui.h> 

#include "ikartPlanner.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

void PlannerThread::run()
{
   /* Bottle &b=port_commands_output.prepare();
    b.clear();
    b.addInt(2);                // polar commands
    b.addDouble(control[0]);    // angle in deg
    b.addDouble(control[1]);    // lin_vel in m/s
    b.addDouble(control[2]);    // ang_vel in deg/s
    port_commands_output.write();
	*/
	
	if (map.map_with_path!=0)
		map.sendToPort(&port_map_output, map.map_with_path);
	else
		map.sendToPort(&port_map_output, map.processed_map);
}

void PlannerThread::setNewAbsTarget(yarp::sig::Vector target)
{
	cell goal = map.world2cell(target);
	cell start = map.world2cell(target);
	start.x = 150;
	start.y = 150;
	goal.x = target[0];
	goal.y = target[1];
	std::queue<cell> path;
	double t1 = yarp::os::Time::now();
	map.findPath(map.processed_map, start , goal, path);
	double t2 = yarp::os::Time::now();

	std::queue<cell> simpler_path;
	map.simplifyPath(map.processed_map, path, simpler_path);
	printf ("time: %f\n", t2-t1);
	for (int i=0; i<simpler_path.size(); i++)
	{
		cell c = simpler_path._Get_container().at(i);
		printf ("%d %d %d\n",i,c.x, c.y);
	}

	if (map.map_with_path==0)
	{
		map.map_with_path = cvCloneImage(map.processed_map);
	}
	cvCopyImage(map.processed_map,map.map_with_path);
	
	CvScalar color = cvScalar(0,200,0);
	map.drawPath(map.map_with_path, start, path, color); 
	color = cvScalar(0,0,200);
	map.drawPath(map.map_with_path, start, simpler_path, color); 
}

void PlannerThread::setNewRelTarget(yarp::sig::Vector target)
{
}

void PlannerThread::stopMovement()
{
}

void PlannerThread::resumeMovement()
{
}

void PlannerThread::pauseMovement(double d)
{
}

void PlannerThread::printStats()
{
}

string PlannerThread::getNavigationStatus()
{
	string s= "IDLE";
	return s;
}

