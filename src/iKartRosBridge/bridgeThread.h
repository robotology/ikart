/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef BRIDGE_THREAD_H
#define BRIDGE_THREAD_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h> 
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Twist.h>
#include "odometer.h"

using namespace std;
using namespace yarp::os;

class BridgeThread: public yarp::os::RateThread
{
    private:
    int    thread_period;
    
    double ikart_x  ;
    double ikart_y  ;
    double ikart_t  ;
    double ikart_vx ;
    double ikart_vy ;
    double ikart_vt ;
    double distance_traveled ;
    double angle_traveled ;
    
    int    laser_step;
    double last_laser[1080];
    double command_x  ;
    double command_y  ;
    double command_t  ;

    yarp::os::Semaphore mutex_command;
    yarp::os::Semaphore mutex_localiz;
    yarp::os::Semaphore mutex_home;
    
    
    
    class ikart_pose
    {
        public:
        double x;
        double y;
        double t;
        ikart_pose() {x=0; y=0; t=0;}
    };
    ikart_pose ikart_home;
    ikart_pose ikart_current_position;

    protected:
    ros::NodeHandle          *nh;
    ros::Publisher           laser_pub;
    ros::Publisher           odometry_pub;
    ros::Publisher           odometer_pub;
    ros::Subscriber          command_sub;
    tf::TransformBroadcaster *tf_broadcaster;
    tf::TransformListener    *tf_listener;
    ResourceFinder           &rf;
    Property                 iKartCtrl_options;
    BufferedPort<Bottle>     input_laser_port; 
    BufferedPort<Bottle>     input_odometry_port; 
    BufferedPort<Bottle>     input_odometer_port; 
    BufferedPort<Bottle>     output_command_port; 
    BufferedPort<Bottle>     output_localization_port;
    int                      timeout_thread;
    int                      timeout_thread_tot;
    int                      timeout_laser;
    int                      timeout_odometry;
    int                      timeout_odometer;
    int                      timeout_laser_tot;
    int                      timeout_odometry_tot;
    int                      timeout_odometer_tot;
    int                      command_wdt;

    sensor_msgs::LaserScan   scan;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
    
    public:
    
    BridgeThread(unsigned int _period, ResourceFinder &_rf, Property options,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        thread_period = _period;
        ikart_x  = 0.0;
        ikart_y  = 0.0;
        ikart_t  = 0.0;
        ikart_vx = 0.0;
        ikart_vy = 0.0;
        ikart_vt = 0.0;
        command_x = 0.0;
        command_y = 0.0 ;
        command_t = 0.0 ;
        distance_traveled = 0.0;
        angle_traveled    = 0.0;
        timeout_thread   = 0;
        timeout_thread_tot   = 0;
        command_wdt = 100;
        timeout_laser = 0;
        timeout_odometry = 0;
        timeout_odometer = 0;
        timeout_laser_tot = 0;
        timeout_odometry_tot = 0;
        timeout_odometer_tot = 0;
        
        laser_step = rf.check("laser_resample",Value(1)).asInt();
        printf ("Using %d laser measurments each scan (max: 1080).\n", 1080/laser_step);
    }

    void setHome();  
    void setHome(double x, double y, double angle);
    void getHome(double &x, double &y, double &angle);  
    int  setGoal(double x, double y, double angle);
    int  navigationStop();
    int  getGoal(double &x, double &y, double &angle);
    string getNavigationStatus();
    void getLocalizedPos(double &x, double &y, double &angle);
    void printStats();
    
    // void stringCallback(const std_msgs::StringConstPtr& message) {}
    
    void commandCallback(const geometry_msgs::Twist& event)
    {
        mutex_command.wait();
        command_x = -event.linear.y;
        command_y = +event.linear.x;
        command_t = -event.angular.z*180.0/M_PI;
        command_wdt = 100;
        mutex_command.post();
    }

    virtual bool threadInit()
    {
        int argc = 0;
        char** argv = 0;
        ros::init (argc, argv, "ikart_ros_bridge");
        nh = new ros::NodeHandle();
        ros::Time::init();
        laser_pub      = nh->advertise<sensor_msgs::LaserScan>         ("/ikart_ros_bridge/laser_out",     1);
        odometer_pub   = nh->advertise<ikart_ros_bridge::odometer>     ("/ikart_ros_bridge/odometer_out",  1);
        odometry_pub   = nh->advertise<nav_msgs::Odometry>             ("/ikart_ros_bridge/odometry_out",  1);
        tf_broadcaster = new tf::TransformBroadcaster;
        tf_listener    = new tf::TransformListener;
        ac             = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true);

        //wait for the action server to come up  
        /*while(!ac->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }*/

        //subscribe ros topics, open yarp ports and make connections
        command_sub = nh->subscribe("cmd_vel", 1, &BridgeThread::commandCallback, this);

        input_laser_port.open("/ikart_ros_bridge/laser:i");
        input_odometry_port.open("/ikart_ros_bridge/odometry:i");
        input_odometer_port.open("/ikart_ros_bridge/odometer:i");
        output_command_port.open("/ikart_ros_bridge/command:o");
        output_localization_port.open("/ikart_ros_bridge/localization:o");

        bool conn_lsr = Network::connect ("/ikart/laser:o","/ikart_ros_bridge/laser:i");
        bool conn_odm = Network::connect ("/ikart/odometry:o","/ikart_ros_bridge/odometry:i");
        bool conn_odt = Network::connect ("/ikart/odometer:o","/ikart_ros_bridge/odometer:i");
        
        if (!conn_lsr || !conn_odm || !conn_odt)
        {
            printf("Connection to iKartCtrl failed\n");
        }

        //prepare here the laser scan message, to save time during the main loop
        int num_readings = 1080/laser_step;
        int laser_frequency = 1080/laser_step;        
        scan.header.frame_id = "base_laser";
        scan.angle_min = -2.35619;
        scan.angle_max =  2.35619;
        scan.angle_increment = 4.7123889 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 10; //100m 
        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);    
        for (int i=0; i< 1080; i=i+laser_step)
        {
            last_laser[i] = 0.0;
            scan.ranges[i] = 0.0;
            scan.intensities[i]=101;
        }
         
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Bridge thread started successfully\n");
        else
            printf("Bridge thread did not start\n");
    }

    virtual void run()
    {
        //********************************************* TIMEOUT CHECK ******************************************
        static double wdt_old=Time::now();
        double wdt=Time::now();
        if (wdt-wdt_old > double(thread_period)/1000.0 + 0.10) 
        {
            timeout_thread++;
            timeout_thread_tot++;
        }     
        //printf("%f %f\n",wdt-wdt_old , double(thread_period)/1000.0 + 0.010);
        wdt_old=wdt;

        //********************************************* LASER PART *********************************************
        Bottle *laser_bottle = 0;
        laser_bottle = input_laser_port.read(false);

        ros::Time now = ros::Time::now();
        scan.header.stamp.sec = now.sec;
        scan.header.stamp.nsec = now.nsec;

        if (laser_bottle)
        {
           for(int j=0; j<1080; j=j+laser_step)
           {
                last_laser[j] = laser_bottle->get(j).asDouble();
                scan.ranges[j]=last_laser[j];
                //scan.intensities[j]=101;
           }
        }
        else  
        {    
           timeout_laser++;
           timeout_laser_tot++;
        }

        laser_pub.publish (scan);

        //********************************************* CREATE NEW TF *********************************************
        tf::StampedTransform laser_trans(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.245,0.0,0.2)),now, "base_link", "base_laser");
        tf_broadcaster->sendTransform(laser_trans);

        mutex_home.wait();
        tf::StampedTransform home_trans(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(ikart_home.x,ikart_home.y,0.0)),now, "map", "home");
        tf_broadcaster->sendTransform(home_trans);
        mutex_home.post();
        
        //********************************************* READ TF      **********************************************
        tf::StampedTransform stamp_loc_trans;
        bool loc_running = tf_listener->canTransform ("/home", "/base_link", ros::Time(0), NULL); 
      
        if (loc_running)
        {       
            // The following try-catch has been added just for teaching purposues, since the block is
            // already protected by the tf_listener->canTransform() check.
            try
            {
                 tf_listener->lookupTransform("/home", "/base_link", ros::Time(0), stamp_loc_trans);
            }
            catch (tf::TransformException ex)
            {
                 ROS_ERROR("%s",ex.what()); 
            }

            geometry_msgs::TransformStamped loc_trans;
            tf::transformStampedTFToMsg (stamp_loc_trans, loc_trans);
            
            mutex_localiz.wait();
            ikart_current_position.x = loc_trans.transform.translation.y;
            ikart_current_position.y = -loc_trans.transform.translation.x;
            ikart_current_position.t = -tf::getYaw(loc_trans.transform.rotation)*180/M_PI;
            mutex_localiz.post();
            
            Bottle &m = output_localization_port.prepare();
            m.clear();
            m.addDouble(ikart_current_position.x);
            m.addDouble(ikart_current_position.y);
            m.addDouble(ikart_current_position.t);
            output_localization_port.write();
        }
         
        //********************************************* COMMAND PART  *********************************************
        mutex_command.wait();        
        command_wdt--;
        if (command_wdt<0)
        {
            //if no commands are received, than turn off control
            command_x = 0;
            command_y = 0;
            command_t = 0;
            command_wdt = 0;
        }
        Bottle &b=output_command_port.prepare();
        b.clear();
        b.addInt(3);
        b.addDouble(command_x);
        b.addDouble(command_y);
        b.addDouble(command_t);
        output_command_port.write();
        mutex_command.post();

        //********************************************* ODOMETER PART *********************************************
        Bottle *odometer_bottle = 0;
        odometer_bottle = input_odometer_port.read(false);
        if (odometer_bottle)
        {
            distance_traveled = odometer_bottle->get(0).asDouble();
            angle_traveled = odometer_bottle->get(1).asDouble();
        }
        else
        {
            timeout_odometer++;
            timeout_odometer_tot++;
        }        
        ikart_ros_bridge::odometer odometer_msg;
        odometer_msg.distance=distance_traveled;
        odometer_msg.angle=angle_traveled;
        odometer_pub.publish(odometer_msg);

        //********************************************* ODOMETRY PART *********************************************
        Bottle *odometry_bottle = 0;
        odometry_bottle = input_odometry_port.read(false);

//#define ODOMETRY_DEBUG
#ifdef ODOMETRY_DEBUG
        ikart_x  = 7;  //m
        ikart_y  = 5;  //m
        ikart_t  = 30; //deg
        ikart_vx = 0;
        ikart_vy = 0;
        ikart_vt = 0;
#else	
        if (odometry_bottle)
        {
            ikart_x  = odometry_bottle->get(1).asDouble();
            ikart_y  = -odometry_bottle->get(0).asDouble();
            ikart_t  = -odometry_bottle->get(2).asDouble();
            ikart_vx = odometry_bottle->get(4).asDouble();
            ikart_vy = -odometry_bottle->get(3).asDouble();
            ikart_vt = -odometry_bottle->get(5).asDouble();
        }
        else
        {
            timeout_odometry++;
            timeout_odometry_tot++;
        }

#endif
        geometry_msgs::Quaternion odom_quat= tf::createQuaternionMsgFromYaw(ikart_t/180.0*M_PI);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp.sec = now.sec;
        odom_trans.header.stamp.nsec = now.nsec;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = ikart_x;
        odom_trans.transform.translation.y = ikart_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        tf_broadcaster->sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp.sec = now.sec;
        odom.header.stamp.nsec = now.nsec;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = ikart_x;
        odom.pose.pose.position.y = ikart_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = ikart_vx;
        odom.twist.twist.linear.y = ikart_vy;
        odom.twist.twist.angular.z = ikart_vt/180.0*M_PI;
        odometry_pub.publish (odom);

        ros::spinOnce (); //@@@@@@@@@@
    }

    virtual void threadRelease()
    {
        input_laser_port.interrupt();
        input_laser_port.close();
        input_odometry_port.interrupt();
        input_odometry_port.close();
        input_odometer_port.interrupt();
        input_odometer_port.close();
        output_command_port.interrupt();
        output_command_port.close();
        output_localization_port.interrupt();
        output_localization_port.close();        
    }

};

#endif
