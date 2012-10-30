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

#ifndef PLANNER_THREAD_H
#define PLANNER_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/RpcClient.h>
#include <string>

#include "status.h"
#include "map.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class PlannerThread: public yarp::os::RateThread
{
    public:
    double goal_tolerance_lin;      //m 
    double goal_tolerance_ang;      //deg
    double waypoint_tolerance_lin;  //m 
    double waypoint_tolerance_ang;  //deg

    protected:
    //configuration parameters
    double    robot_radius;     //m
    map_class map;

    //ports
    BufferedPort<yarp::sig::Vector>                        port_localization_input;
    BufferedPort<yarp::os::Bottle>                         port_status_input;
    BufferedPort<yarp::sig::Vector>                        port_laser_input;
    BufferedPort<yarp::os::Bottle>                         port_yarpview_target_input;

    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > port_map_output;
    BufferedPort<yarp::os::Bottle>                         port_status_output;
    RpcClient                                              port_commands_output;

    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   target_data;
    yarp::sig::Vector   laser_data;
    int                 loc_timeout_counter;
    int                 inner_status_timeout_counter;

    std::queue<cell>    current_path;
    status_type         planner_status;
    status_type         inner_status;

    public:
    PlannerThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        planner_status = IDLE;
        inner_status = IDLE;
        localization_data.resize(3,0.0);
        laser_data.resize(1080,1000.0);
        loc_timeout_counter = 0;
        inner_status_timeout_counter = 0;
        goal_tolerance_lin = 0.05;
        goal_tolerance_ang = 0.6;
        waypoint_tolerance_lin = 0.05;
        waypoint_tolerance_ang = 0.6;
    }

    virtual bool threadInit()
    {
        //read configuration parametes
        yarp::os::ResourceFinder mapFinder;
        string default_map_path = "iKart/maps";

        mapFinder.setDefaultContext(default_map_path.c_str());
        mapFinder.configure("ICUB_ROOT",0,0);

        string map_filename;
        map_filename = mapFinder.getContextPath().c_str() + string("/");
        map_filename = map_filename + rf.find("map_file").asString().c_str();
        //map_filename = rf.find("map_file").asString().c_str();
        if (!map.loadMap(map_filename))
        {
            printf("map file not found, closing\n");
            return false;
        }

        if (rf.check("waypoint_tolerance_lin")) {waypoint_tolerance_lin = rf.find("waypoint_tolerance_lin").asDouble();}
        if (rf.check("waypoint_tolerance_ang")) {waypoint_tolerance_ang = rf.find("waypoint_tolerance_ang").asDouble();}
        if (rf.check("goal_tolerance_lin"))     {goal_tolerance_lin = rf.find("goal_tolerance_lin").asDouble();}
        if (rf.check("goal_tolerance_ang"))     {goal_tolerance_ang = rf.find("goal_tolerance_ang").asDouble();}

        //open module ports
        string localName = "/ikartPathPlanner";
        port_localization_input.open((localName+"/localization:i").c_str());
        port_laser_input.open((localName+"/laser:i").c_str());
        port_status_input.open((localName+"/navigationStatus:i").c_str());
        port_status_output.open((localName+"/plannerStatus:o").c_str());
        port_commands_output.open((localName+"/commands:o").c_str());
        port_map_output.open((localName+"/map:o").c_str());
        port_yarpview_target_input.open((localName+"/yarpviewTarget:i").c_str());

        //automatic port connections
        bool b = false;
        b = Network::connect("/ikart_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
        // if (!b) {fprintf (stderr,"Unable to connect the localization port!\n"); return false;}
        b = Network::connect("/ikart/laser:o",(localName+"/laser:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the laser port!\n"); }
        return true;
    }

    virtual void run();

    void setNewAbsTarget(yarp::sig::Vector target);
    void setNewRelTarget(yarp::sig::Vector target);
    void sendWaypoint();
    void startNewPath(cell target);
    void stopMovement();
    void pauseMovement (double secs);
    void resumeMovement();
    string getNavigationStatus();

    virtual void threadRelease()
    {    
        port_localization_input.interrupt();
        port_localization_input.close();
        port_laser_input.interrupt();
        port_laser_input.close();
        port_map_output.interrupt();
        port_map_output.close();
        port_status_input.interrupt();
        port_status_input.close();
        port_status_output.interrupt();
        port_status_output.close();
        port_commands_output.interrupt();
        port_commands_output.close();
        port_yarpview_target_input.interrupt();
        port_yarpview_target_input.close();
    }

    void printStats();

};

#endif
