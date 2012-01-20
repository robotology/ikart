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

#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

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
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

#include "odometry.h"
#include "motors.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define MAX_LINEAR_VEL  0.4  // maximum linear  velocity (m/s)
#define MAX_ANGULAR_VEL 24.0 // maximum angular velocity (deg/s)

class ControlThread : public yarp::os::RateThread
{
private:
    Property iKartCtrl_options;

    enum
    {
        IKART_CONTROL_NONE = 0,
        IKART_CONTROL_OPENLOOP = 1,
        IKART_CONTROL_SPEED = 2
    };
 
    double              thread_period;

    int                 thread_timeout_counter;

    //movement control variables (internally computed)
    double              exec_linear_speed;
    double              exec_angular_speed;
    double              exec_desired_direction;
    double              exec_pwm_gain;

    //ikart pids
    bool                        pid_control_enabled;
    iCub::ctrl::parallelPID*    linear_speed_pid;
    iCub::ctrl::parallelPID*    angular_speed_pid;
    iCub::ctrl::parallelPID*    direction_pid;
    double                      pidout_linear_speed;
    double                      pidout_angular_speed;
    double                      pidout_direction;
    double                      feedback_linear_speed;
    double                      feedback_angular_speed;
    double                      feedback_desired_direction;

    //controller parameters
    double              lin_ang_ratio;
    bool                both_lin_ang_enabled;

protected:
    ResourceFinder            &rf;
    PolyDriver                *control_board_driver;
    BufferedPort<Bottle>      port_movement_control;
    BufferedPort<Bottle>      port_auxiliary_control;
    BufferedPort<Bottle>      port_joystick_control;

    BufferedPort<Bottle>      port_debug_direction;
    BufferedPort<Bottle>      port_debug_linear;
    BufferedPort<Bottle>      port_debug_angular;

    Odometry*                 odometry_handler;
    MotorControl*             motor_handler;
        
    bool   filter_enabled;

    string remoteName;
    string localName;

    IPidControl       *ipid;
    IVelocityControl  *ivel;
    IEncoders         *ienc;
    IAmplifierControl *iamp;
    IOpenLoopControl  *iopl;
    IControlMode      *icmd;

public:
    Odometry* const     get_odometry_handler() {return odometry_handler;}
    MotorControl* const get_motor_handler()    {return motor_handler;}

    ControlThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        control_board_driver       = 0;
        thread_timeout_counter     = 0;

        pid_control_enabled = false;
        filter_enabled = true;
        lin_ang_ratio = 0.7;
        both_lin_ang_enabled = true;
        thread_period = _period;

        linear_speed_pid=0;
        angular_speed_pid=0;
        direction_pid=0;
        remoteName = iKartCtrl_options.find("remote").asString();
        localName = iKartCtrl_options.find("local").asString();
    }

    virtual bool threadInit()
    {
       // open the control board driver
        printf("\nOpening the motors interface...\n");
        int trials=0;
        double start_time = yarp::os::Time::now();
        Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote",remoteName.c_str());
        control_board_options.put("local",localName.c_str());

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (control_board_driver)
            {
                delete control_board_driver;
                control_board_driver=0;
            }

            //creates the new device driver
            control_board_driver=new PolyDriver(control_board_options);
            bool connected =control_board_driver->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (10s) is expired
            if (current_time-start_time > 10.0)
            {
                fprintf(stderr,"It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                if (control_board_driver)
                {
                    delete control_board_driver;
                    control_board_driver=0;
                }
                return false;
            }

            yarp::os::Time::delay(0.5);
            trials++;
            fprintf(stderr,"\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        odometry_handler = new Odometry (thread_period,rf,iKartCtrl_options,control_board_driver);
        motor_handler = new MotorControl(thread_period,rf,iKartCtrl_options,control_board_driver);
        odometry_handler->open();
        motor_handler->open();

        //create the pid controllers
        pid_control_enabled =  (bool)(iKartCtrl_options.check("pid_control_enabled",Value(0),"enable the high level PID control on linear speed, angular speed and direction.").asInt());
        yarp::sig::Vector kp[3],ki[3],kd[3];
        yarp::sig::Vector wp[3],wi[3],wd[3];
        yarp::sig::Vector N[3];
        yarp::sig::Vector Tt[3];
        yarp::sig::Matrix sat[3];
        for (int i=0; i<3; i++)
        {
            kp[i].resize(1); ki[i].resize(1); kd[i].resize(1);
            wp[i].resize(1); wi[i].resize(1); wd[i].resize(1);
            N[i].resize(1); Tt[i].resize(1); sat[i].resize(1,2);
        }
        linear_speed_pid  = new iCub::ctrl::parallelPID(thread_period,kp[0],ki[0],kd[0],wp[0],wi[0],wd[0],N[0],Tt[0],sat[0]);
        angular_speed_pid = new iCub::ctrl::parallelPID(thread_period,kp[1],ki[1],kd[1],wp[1],wi[1],wd[1],N[1],Tt[1],sat[1]);
        direction_pid     = new iCub::ctrl::parallelPID(thread_period,kp[2],ki[2],kd[2],wp[2],wi[2],wd[2],N[2],Tt[2],sat[2]);

        //debug ports
        if (1)
        {
            port_debug_direction.open((localName+"/debug/direction:o").c_str());
            port_debug_linear.open((localName+"/debug/linear:o").c_str());
            port_debug_angular.open((localName+"/debug/angular:o").c_str());
        }

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Control thread started successfully\n");
        else
            printf("Control thread did not start\n");
    }

    double lp_filter_1Hz(double input, int i)
    {
       //This is a butterworth low pass first order, with a cut off freqency of 1Hz
       //It must be used with a sampling frequency of 50Hz (20ms)
       static double xv[2][10], yv[2][10];
       xv[0][i] = xv[1][i]; 
       xv[1][i] = input /1.689454484e+01;
       yv[0][i] = yv[1][i]; 
       yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.8816185924 * yv[0][i]);
       return yv[1][i];
    }
    double lp_filter_0_5Hz(double input, int i)
    {
        //This is a butterworth low pass first order, with a cut off freqency of 0.5Hz
        //It must be used with a sampling frequency of 50Hz (20ms)
       static double xv[2][10], yv[2][10];
       xv[0][i] = xv[1][i]; 
       xv[1][i] = input /3.282051595e+01;
       yv[0][i] = yv[1][i]; 
       yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.9390625058 * yv[0][i]);
       return yv[1][i];
    }

    double ratelim_filter_0(double input, int i)
    {
        //This is a rate limiter filter. 
        static double prev[3];
        if      (input>prev[i]+10) prev[i]=prev[i]+10;
        else if (input<prev[i]-10) prev[i]=prev[i]-10;
        else     prev[i]=input;
        return prev[i];
    }

    virtual void run()
    {
        this->odometry_handler->compute();

        double              linear_speed=0;
        double              angular_speed=0;
        double              desired_direction=0;
        double              pwm_gain=0;
        this->motor_handler->read_inputs(&linear_speed, &angular_speed, &desired_direction, &pwm_gain);

        //saturators
        double MAX_VALUE = 0;
        if (motor_handler->get_ikart_control_type() == IKART_CONTROL_OPENLOOP)
        {
            MAX_VALUE = 1333; // Maximum joint PWM
        }
        else if (motor_handler->get_ikart_control_type() == IKART_CONTROL_SPEED)
        {
            MAX_VALUE = 200; // Maximum joint speed (deg/s)
        }

        //Here we suppouse that values are already in the format 0-100%
        exec_linear_speed = linear_speed / 100.0 * MAX_VALUE;
        exec_angular_speed = angular_speed / 100.0 * MAX_VALUE;
        exec_pwm_gain = pwm_gain / 100.0 * 1.0;
        exec_desired_direction = desired_direction;

        if (motor_handler->get_ikart_control_type() == IKART_CONTROL_OPENLOOP)
        {
            if (lin_ang_ratio<0.0)  lin_ang_ratio = 0.0;
            if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
            if (exec_linear_speed  >  MAX_VALUE*lin_ang_ratio) exec_linear_speed  = MAX_VALUE*lin_ang_ratio;
            if (exec_linear_speed  < -MAX_VALUE*lin_ang_ratio) exec_linear_speed  = -MAX_VALUE*lin_ang_ratio;
            if (exec_angular_speed >  MAX_VALUE*(1-lin_ang_ratio)) exec_angular_speed = MAX_VALUE*(1-lin_ang_ratio);
            if (exec_angular_speed < -MAX_VALUE*(1-lin_ang_ratio)) exec_angular_speed = -MAX_VALUE*(1-lin_ang_ratio);
        }

        double pidout_linear_speed = 0;
        double pidout_angular_speed = 0;
        double pidout_direction = 0;

        if (pid_control_enabled==true)
        {
            //feedback_linear_speed = 
            yarp::sig::Vector tmp;
            tmp = linear_speed_pid->compute(yarp::sig::Vector(1,&exec_linear_speed),yarp::sig::Vector(1,&feedback_linear_speed));
            pidout_linear_speed  = exec_pwm_gain * tmp[0];
            tmp = angular_speed_pid->compute(yarp::sig::Vector(1,&exec_angular_speed),yarp::sig::Vector(1,&feedback_angular_speed));
            pidout_angular_speed = exec_pwm_gain * tmp[0];
            tmp = direction_pid->compute(yarp::sig::Vector(1,&exec_desired_direction),yarp::sig::Vector(1,&feedback_desired_direction));
            pidout_direction     = tmp[0];
            Bottle &b1=port_debug_linear.prepare();
            b1.clear();
            b1.addDouble(exec_linear_speed);
            b1.addDouble(feedback_linear_speed);
            //b1.addDouble(exec_linear_speed-feedback_linear_speed);
            port_debug_linear.write();
            Bottle &b2=port_debug_angular.prepare();
            b2.clear();
            b2.addDouble(exec_angular_speed);
            b2.addDouble(feedback_angular_speed);
            b2.addDouble(exec_angular_speed-feedback_angular_speed);
            port_debug_angular.write();
            Bottle &b3=port_debug_direction.prepare();
            b3.clear();
            b3.addDouble(exec_desired_direction);
            b3.addDouble(feedback_desired_direction);
            b3.addDouble(exec_desired_direction-feedback_desired_direction);
            port_debug_direction.write();
        }
        else
        {
            pidout_linear_speed  = exec_pwm_gain * exec_linear_speed;
            pidout_angular_speed = exec_pwm_gain * exec_angular_speed;
            pidout_direction     = exec_desired_direction;
        }

        this->motor_handler->execute(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }

    virtual void threadRelease()
    {
        if (odometry_handler)  {delete odometry_handler; odometry_handler=0;}
        if (motor_handler)     {delete motor_handler; motor_handler=0;}

        if (linear_speed_pid)  {delete linear_speed_pid;  linear_speed_pid=0;}
        if (angular_speed_pid) {delete angular_speed_pid; angular_speed_pid=0;}
        if (direction_pid)     {delete direction_pid;     direction_pid=0;}

        if (1)
        {
            port_debug_linear.interrupt();
            port_debug_linear.close();
            port_debug_angular.interrupt();
            port_debug_angular.close();
            port_debug_direction.interrupt();
            port_debug_direction.close();
        }
    }
};

#endif
