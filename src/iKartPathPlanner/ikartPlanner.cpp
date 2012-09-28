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
	//read the localization data
	yarp::sig::Vector *loc = port_localization_input.read(false);
	if (loc)
	{
		localization_data = *loc;
	    loc_timeout_counter=0;
	}
    else loc_timeout_counter++;

	//read the internal navigation status
	yarp::os::Bottle *st = port_status_input.read(false);
	if (st)
	{
		string s = st->get(0).toString();
		inner_status_timeout_counter=0;
		inner_status = IDLE; //convet s to inner status
	}
    else inner_status_timeout_counter++;

	//check if the next waypoint has to be sent
	if (planner_status == MOVING)
	{
		if (inner_status == REACHED)
		{
			if (current_path.size() == 0)
			{
				//navigation is complete
				printf ("navigation complete\n");
				planner_status = REACHED;
			}
			else
			{
				//get the next waypoint from the list
				cell waypoint = current_path.front();
				current_path.pop();
				//send the waypoint to the inner controller
				Bottle &b=this->port_status_output.prepare();
				b.clear();
				b.addString("gotoAbs"); 
				b.addDouble(waypoint.x);
				b.addDouble(waypoint.y);
				if (current_path.size()==1 && target_data.size()==3)
				{
					//add the orientation to the last waypoint
					b.addDouble(target_data[3]);
				}
				port_status_output.write();
			}
		}
		else if (inner_status == MOVING)
		{
			//do nothing, just wait
		}
		else if (inner_status == ABORTED)
		{
			//terminate navigation
			planner_status = ABORTED;
			printf ("unable to reach next waypoint, aborting navigation\n");
			//current_path.clear();
		}
	}
	else if (planner_status == REACHED)
	{
		//do nothing, just wait
	}
	else if (planner_status == IDLE)
	{
		//do nothing, just wait
	}
	else
	{
		//unknown status
		printf ("unknown status:%d\n", planner_status);
	}

	//broadcast the planner status
	if (port_status_output.getOutputCount()>0)
	{
		string s;
		Bottle &b=this->port_status_output.prepare();
		b.clear();
		b.addString(s.c_str());    
		port_status_output.write();
	}
	
	//draw the map
	cell start = map.world2cell(localization_data);
	start.x = 150;//&&&&& 150
	start.y = 150;//&&&&&

	static IplImage* map_with_path = 0;
	if (map_with_path==0) map_with_path = cvCloneImage(map.processed_map);
	else cvCopyImage(map.processed_map,map_with_path);
	
	CvScalar color = cvScalar(0,200,0);
	map.drawPath(map_with_path, start, current_path, color); 

	static IplImage* map_with_location = 0;
	static CvScalar blue_color = cvScalar(0,0,200);
	if (map_with_location == 0) map_with_location = cvCloneImage(map_with_path);
	else cvCopyImage(map_with_path, map_with_location);
	
	map.drawCurrentPosition(map_with_location,start,blue_color);
	
	map.sendToPort(&port_map_output,map_with_location);
}

void PlannerThread::setNewAbsTarget(yarp::sig::Vector target)
{
	cell goal = map.world2cell(target);
	cell start = map.world2cell(localization_data);
	start.x = 150;//&&&&&
	start.y = 150;//&&&&&
	target_data = target;
	goal.x = target_data[0];
	goal.y = target_data[1];
	double t1 = yarp::os::Time::now();
	std::queue<cell> empty;
    std::swap(current_path, empty );
	bool b = map.findPath(map.processed_map, start , goal, current_path);
	if (!b)
	{
		printf ("path not found\n");
		return;
	}
	double t2 = yarp::os::Time::now();

	std::queue<cell> simpler_path;
	//map.simplifyPath(map.processed_map, current_path, simpler_path);
	//current_path = simpler_path;

	printf ("time: %f\n", t2-t1);
	for (int i=0; i<simpler_path.size(); i++)
	{
		cell c = simpler_path._Get_container().at(i);
		printf ("%d %d %d\n",i,c.x, c.y);
	}
}

void PlannerThread::setNewRelTarget(yarp::sig::Vector target)
{
	target_data = target;
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

