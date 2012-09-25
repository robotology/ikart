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
#include <string>

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

    protected:
    //configuration parameters
    double    robot_radius;     //m
	map_class map;

    //ports
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_map_output;
	BufferedPort<yarp::sig::Vector>   port_localization_input;
	BufferedPort<yarp::sig::Vector>   port_target_input;
    BufferedPort<yarp::sig::Vector>   port_laser_input;
    BufferedPort<yarp::os::Bottle>    port_commands_output;

    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   target_data;
    yarp::sig::Vector   laser_data;
	enum                status_type {IDLE=0, MOVING, WAITING_OBSTACLE, REACHED, ABORTED} status;
    int                 timeout_counter;
    int                 retreat_counter;
	double              obstacle_time;

    public:
    PlannerThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        status = IDLE;
        timeout_counter     = 0;
        localization_data.resize(3,0.0);
        target_data.resize(3,0.0);
        laser_data.resize(1080,1000.0);
    }

    virtual bool threadInit()
    {
        //read configuration parametes
		string map_filename = rf.find("map_file").asString();
		if (!map.loadMap(map_filename))
		{
			printf("map file not found, closing\n");
			return false;
		}

        //open module ports
		string localName = "/ikartPathPlanner";
        port_localization_input.open((localName+"/localization:i").c_str());
		port_laser_input.open((localName+"/laser:i").c_str());
		port_map_output.open((localName+"/map:o").c_str());

        //automatic port connections
        bool b = false;
        b = Network::connect("/ikart_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
       // if (!b) {fprintf (stderr,"Unable to connect the localization port!"); return false;}
        b = Network::connect("/ikart/laser:o",(localName+"/laser:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the laser port!"); }
        return true;
    }

    virtual void run();

	void setNewAbsTarget(yarp::sig::Vector target);
	void setNewRelTarget(yarp::sig::Vector target);
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
    }

    void printStats();

};

#endif
