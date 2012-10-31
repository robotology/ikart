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

#include "status.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class GotoThread: public yarp::os::RateThread
{
    private:
    void sendOutput();

    public:
    bool   enable_stop_on_obstacles;
    bool   enable_retreat;
    double goal_tolerance_lin;  //m 
    double goal_tolerance_ang;  //deg

    public:
    //configuration parameters
    double k_ang_gain;
    double k_lin_gain;
    double max_lin_speed;       //m/s
    double max_ang_speed;       //deg/s
    double min_lin_speed;       //m/s
    double min_ang_speed;       //deg/s
    double robot_radius;        //m
    int    retreat_duration; 

    protected:
    //pause info
    double pause_start;
    double pause_duration;

    //ports
    BufferedPort<yarp::sig::Vector> port_odometry_input;
    BufferedPort<yarp::sig::Vector> port_localization_input;
    BufferedPort<yarp::sig::Vector> port_target_input;
    BufferedPort<yarp::sig::Vector> port_laser_input;
    BufferedPort<yarp::os::Bottle>  port_commands_output;
    BufferedPort<yarp::os::Bottle>  port_status_output;

    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   odometry_data;
    yarp::sig::Vector   target_data;
    yarp::sig::Vector   laser_data;
    yarp::sig::Vector   control_out;
    status_type         status;
    int                 loc_timeout_counter;
    int                 odm_timeout_counter;
    int                 retreat_counter;
    double              obstacle_time;

    public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        status = IDLE;
        loc_timeout_counter = 0;
        odm_timeout_counter = 0;
        localization_data.resize(3,0.0);
        target_data.resize(3,0.0);
        laser_data.resize(1080,1000.0);
        retreat_counter = 0;
        enable_stop_on_obstacles = true;
        control_out.resize(3,0.0);
        pause_start = 0;
        pause_duration = 0;
        goal_tolerance_lin = 0.05;
        goal_tolerance_ang = 0.6;
    }

    virtual bool threadInit()
    {
        //read configuration parametes
        k_ang_gain = 0.05;
        k_lin_gain = 0.1;
        max_lin_speed = 0.9;  //m/s
        max_ang_speed = 10.0; //deg/s
        min_lin_speed = 0.0;  //m/s
        min_ang_speed = 0.0; //deg/s
        robot_radius = 0.30;  //m
        printf ("Using following paramters:\n %s\n", rf.toString().c_str());
        if (rf.check("ang_speed_gain"))     {k_ang_gain = rf.find("ang_speed_gain").asDouble();}
        if (rf.check("lin_speed_gain"))     {k_lin_gain = rf.find("lin_speed_gain").asDouble();}
        if (rf.check("max_lin_speed"))      {max_lin_speed = rf.find("max_lin_speed").asDouble();}
        if (rf.check("max_ang_speed"))      {max_ang_speed = rf.find("max_ang_speed").asDouble();}
        if (rf.check("min_lin_speed"))      {min_lin_speed = rf.find("min_lin_speed").asDouble();}
        if (rf.check("min_ang_speed"))      {min_ang_speed = rf.find("min_ang_speed").asDouble();}
        if (rf.check("robot_radius"))       {robot_radius = rf.find("robot_radius").asDouble();}
        if (rf.check("goal_tolerance_lin")) {goal_tolerance_lin = rf.find("goal_tolerance_lin").asDouble();}
        if (rf.check("goal_tolerance_ang")) {goal_tolerance_ang = rf.find("goal_tolerance_ang").asDouble();}

        enable_retreat = false;
        retreat_duration = 300;

        //open module ports
        string localName = "/ikartGoto";
        port_localization_input.open((localName+"/localization:i").c_str());
        port_target_input.open((localName+"/target:i").c_str());
        port_laser_input.open((localName+"/laser:i").c_str());
        port_commands_output.open((localName+"/control:o").c_str());
        port_status_output.open((localName+"/status:o").c_str());
        port_odometry_input.open((localName+"/odometry:i").c_str());

        //automatic port connections
        /*bool b = false;
        b = Network::connect("/ikart_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the localization port!"); return false;}
        b = Network::connect((localName+"/commands:o").c_str(),"/ikart/control:i", "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the output command port!"); return false;}
        b = Network::connect("/ikart/laser:o",(localName+"/laser:i").c_str(), "udp", false);
        if (!b) {fprintf (stderr,"Unable to connect the laser port!"); }*/
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
        port_target_input.interrupt();
        port_target_input.close();
        port_laser_input.interrupt();
        port_laser_input.close();
        port_commands_output.interrupt();
        port_commands_output.close();
        port_status_output.interrupt();
        port_status_output.close();
        port_odometry_input.interrupt();
        port_odometry_input.close();
    }

    void printStats();
    bool check_obstacles_in_path();

};

#endif
