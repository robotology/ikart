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

status_type string2status(string s)
{
    //enum status_type {IDLE=0, MOVING, WAITING_OBSTACLE, REACHED, ABORTED, PAUSED};
    status_type status;
    if      (s=="IDLE")     status = IDLE;
    else if (s=="MOVING")   status = MOVING;
    else if (s=="WAITING_OBSTACLE")  status = WAITING_OBSTACLE;
    else if (s=="REACHED")  status = REACHED;
    else if (s=="ABORTED")  status = ABORTED;
    else if (s=="PAUSED")   status = PAUSED;
    else 
    {
        printf ("ERROR: unknown status of inner controller!");
        status = IDLE;
    }
    return status;
}

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
        string s = st->get(0).toString().c_str();
        inner_status_timeout_counter=0;
        //convet s to inner status
        inner_status = string2status(s);
    }
    else inner_status_timeout_counter++;

    //check if the next waypoint has to be sent
    int path_size = current_path.size();
    if (planner_status == MOVING)
    {
        if (inner_status == REACHED)
        {
            if (path_size == 0)
            {
                //navigation is complete
                printf ("navigation complete\n");
                planner_status = REACHED;
            }
            else if (path_size == 1)
            {
                 //send the next waypoint
                printf ("sending the last waypoint\n");
                sendWaypoint();
            }
            else
            {
                //send the next waypoint
                 printf ("sending the next waypoint\n");
                sendWaypoint();
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
        else if (inner_status == IDLE)
        {
            //send the first waypoint
            printf ("sending the first waypoint\n");
            sendWaypoint();
        }
        else
        {
            printf ("unrecognized inner status: %d\n", inner_status.getStatusAsInt());
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
        printf ("unknown status:%d\n", planner_status.getStatusAsInt());
    }

    //broadcast the planner status
    if (port_status_output.getOutputCount()>0)
    {
        string s = planner_status.getStatusAsString();
        Bottle &b=this->port_status_output.prepare();
        b.clear();
        b.addString(s.c_str());
        port_status_output.write();
    }
    
    //draw the map
    cell start = map.world2cell(localization_data);
    //start.x = 150;//&&&&& 150
    //start.y = 150;//&&&&&

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

void PlannerThread::sendWaypoint()
{
    int path_size = current_path.size();
    if (path_size==0)
    {
        printf("Path queue is empty!\n");
        planner_status=IDLE;
        return;
    }
    //get the next waypoint from the list
    cell waypoint = current_path.front();
    current_path.pop();
    //send the waypoint to the inner controller
    Bottle cmd1, ans1;
    cmd1.addString("gotoAbs"); 
    yarp::sig::Vector v = map.cell2world(waypoint);
    cmd1.addDouble(v[0]);
    cmd1.addDouble(v[1]);
    if (current_path.size()==1 && target_data.size()==3)
    {
        //add the orientation to the last waypoint
        cmd1.addDouble(target_data[2]);
    }
    printf ("sending command: %s\n", cmd1.toString().c_str());
    port_commands_output.write(cmd1,ans1);
    printf ("received answer: %s\n", ans1.toString().c_str());

    Bottle cmd2, ans2;
    cmd2.addString("get");
    cmd2.addString("navigation_status");
    printf ("sending command: %s\n", cmd2.toString().c_str());
    port_commands_output.write(cmd2,ans2);
    printf ("received answer: %s\n", ans2.toString().c_str());
    inner_status = string2status(ans2.toString().c_str());
}

void PlannerThread::startNewPath(cell target)
{
    cell start = map.world2cell(localization_data);
#ifdef DEBUG_WITH_CELLS
    start.x = 150;//&&&&&
    start.y = 150;//&&&&&
#endif
    double t1 = yarp::os::Time::now();
    std::queue<cell> empty;
    std::swap(current_path, empty );
    bool b = map.findPath(map.processed_map, start , target, current_path);
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
    /*for (int i=0; i<simpler_path.size(); i++)
    {
        cell c = simpler_path._Get_container().at(i);
        printf ("%d %d %d\n",i,c.x, c.y);
    }*/

    //send the tolerance to the inner controller
    Bottle cmd1, ans1;
    cmd1.addString("set"); 
    cmd1.addString("linear_tol");
    cmd1.addDouble(waypoint_tolerance_lin);
    port_commands_output.write(cmd1,ans1);

    Bottle cmd2, ans2;
    cmd2.addString("set"); 
    cmd2.addString("angular_tol");
    cmd2.addDouble(waypoint_tolerance_ang);
    port_commands_output.write(cmd2,ans2);

    //just set the status to moving, do not set position commands.
    //The wayypoint ist set in the main 'run' loop.
    planner_status = MOVING;
}

void PlannerThread::setNewAbsTarget(yarp::sig::Vector target)
{
    if (planner_status != IDLE)
    {
        printf ("Not in idle state, send a 'stop' first\n");
        return;
    }

    target_data = target;
    cell goal = map.world2cell(target);
#ifdef DEBUG_WITH_CELLS
    goal.x = (int)target_data[0]; //&&&&&
    goal.y = (int)target_data[1]; //&&&&&
#endif
    startNewPath(goal);
}

void PlannerThread::setNewRelTarget(yarp::sig::Vector target)
{
    if (planner_status != IDLE)
    {
        printf ("Not in idle state, send a 'stop' first\n");
        return;
    }

    double a = localization_data[2]/180.0*M_PI;
    target_data[0]=target[1] * cos (a) - (-target[0]) * sin (a) + localization_data[0] ;
    target_data[1]=target[1] * sin (a) + (-target[0]) * cos (a) + localization_data[1] ;
    target_data[2]=-target[2] + localization_data[2];
    cell goal = map.world2cell(target);
    startNewPath(goal);
}

void PlannerThread::stopMovement()
{
    if (planner_status != IDLE)
    {
        printf ("Navigation stopped\n");
        planner_status=IDLE;
        yarp::sig::Vector v(3,0.0);
        setNewRelTarget(v);
    }
    else
    {
        printf ("Already not moving\n");
    }
}

void PlannerThread::resumeMovement()
{
    printf ("Not yet implemented\n");
}

void PlannerThread::pauseMovement(double d)
{
    printf ("Not yet implemented\n");
}

void PlannerThread::printStats()
{
}

string PlannerThread::getNavigationStatus()
{
    string s= planner_status.getStatusAsString();
    return s;
}

