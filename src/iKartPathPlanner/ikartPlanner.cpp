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

	std::queue<cell> path;
	cell start (150, 150);
	cell goal  (250, 270);
	path.push(start);
	path.push(goal);
	std::queue<cell> found_path;
	map.findPath(map.processed_map,start,goal,found_path);
	map.drawPath(map.processed_map, start, path); 
	map.sendToPort(&port_map_output);
}

void PlannerThread::setNewAbsTarget(yarp::sig::Vector target)
{
	cell goal = map.world2cell(target);
	cell current_pos = map.world2cell(target);
	std::queue<cell> path;
	map.findPath(map.processed_map, current_pos , goal, path);
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

