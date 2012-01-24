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

#include "controlThread.h"
#include "filters.h"

void ControlThread::run()
{
    this->odometry_handler->compute();

    double              linear_speed=0;
    double              angular_speed=0;
    double              desired_direction=0;

    double              pwm_gain=0;
    this->motor_handler->read_inputs(&linear_speed, &angular_speed, &desired_direction, &pwm_gain);

    //Here we suppouse that values are already in the format 0-100%
    double MAX_VALUE = 0;
    if (motor_handler->get_ikart_control_type() == IKART_CONTROL_OPENLOOP_NO_PID)
    {
        MAX_VALUE = 1333; // Maximum joint PWM
        exec_linear_speed  = linear_speed  / 100.0 * 1300.0;
        exec_angular_speed = angular_speed / 100.0 * 1300.0;
        exec_pwm_gain = pwm_gain / 100.0 * 1.0;
        exec_desired_direction = desired_direction;
        if (lin_ang_ratio<0.0)  lin_ang_ratio = 0.0;
        if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
        if (exec_linear_speed  >  MAX_VALUE*lin_ang_ratio) exec_linear_speed  = MAX_VALUE*lin_ang_ratio;
        if (exec_linear_speed  < -MAX_VALUE*lin_ang_ratio) exec_linear_speed  = -MAX_VALUE*lin_ang_ratio;
        if (exec_angular_speed >  MAX_VALUE*(1-lin_ang_ratio)) exec_angular_speed = MAX_VALUE*(1-lin_ang_ratio);
        if (exec_angular_speed < -MAX_VALUE*(1-lin_ang_ratio)) exec_angular_speed = -MAX_VALUE*(1-lin_ang_ratio);
        double pidout_linear_speed  = exec_pwm_gain * exec_linear_speed;
        double pidout_angular_speed = exec_pwm_gain * exec_angular_speed;
        double pidout_direction     = exec_desired_direction;
        this->motor_handler->execute(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }
    else if (motor_handler->get_ikart_control_type() == IKART_CONTROL_OPENLOOP_PID)
    {
        exec_pwm_gain = pwm_gain / 100.0 * 1.0;
        exec_linear_speed  = linear_speed  * exec_pwm_gain;
        exec_angular_speed = angular_speed * exec_pwm_gain;
        exec_desired_direction = desired_direction;

        double feedback_linear_speed = this->odometry_handler->vel_lin;
        double feedback_angular_speed = this->odometry_handler->vel_theta;
        double feedback_desired_direction = this->odometry_handler->vel_heading;
        yarp::sig::Vector tmp;
        tmp = linear_ol_pid->compute(yarp::sig::Vector(1,&exec_linear_speed),yarp::sig::Vector(1,&feedback_linear_speed));
        double pidout_linear_speed  = exec_pwm_gain * tmp[0];
        tmp = angular_ol_pid->compute(yarp::sig::Vector(1,&exec_angular_speed),yarp::sig::Vector(1,&feedback_angular_speed));
        double pidout_angular_speed = exec_pwm_gain * tmp[0];
        tmp = direction_ol_pid->compute(yarp::sig::Vector(1,&exec_desired_direction),yarp::sig::Vector(1,&feedback_desired_direction));
        double pidout_direction     = tmp[0];
            pidout_linear_speed=0;
            //pidout_angular_speed=0;
            pidout_direction=0;
        this->motor_handler->execute(pidout_linear_speed,pidout_direction,pidout_angular_speed);
            
        if (1) // debug block
        {
            Bottle &b1=port_debug_linear.prepare();
            b1.clear();
            //b1.addDouble(exec_linear_speed);
            //b1.addDouble(feedback_linear_speed);
            b1.addDouble(exec_linear_speed-feedback_linear_speed);
            port_debug_linear.write();
            Bottle &b2=port_debug_angular.prepare();
            b2.clear();
            //b2.addDouble(exec_angular_speed);
            //b2.addDouble(feedback_angular_speed);
            b2.addDouble(exec_angular_speed-feedback_angular_speed);
            port_debug_angular.write();
            Bottle &b3=port_debug_direction.prepare();
            b3.clear();
            //b3.addDouble(exec_desired_direction);
            //b3.addDouble(feedback_desired_direction);
            b3.addDouble(exec_desired_direction-feedback_desired_direction);
            port_debug_direction.write();
        }
    }
    else if (motor_handler->get_ikart_control_type() == IKART_CONTROL_SPEED_NO_PID)
    {
        MAX_VALUE = 200; // Maximum joint speed (deg/s)
        //exec_linear_speed = linear_speed / 100.0 * 0.1/*MAX_VALUE*/;  //0.1 m/s
        //exec_angular_speed = angular_speed / 100.0 * 20/*MAX_VALUE*/; //20 deg/s
        exec_linear_speed = linear_speed / 100.0 * 200.0/*MAX_VALUE*/;  //0.1 m/s
        exec_angular_speed = angular_speed / 100.0 * 200.0/*MAX_VALUE*/; //20 deg/s
        exec_pwm_gain = pwm_gain / 100.0 * 1.0;
        exec_desired_direction = desired_direction;
        double pidout_linear_speed  = exec_pwm_gain * exec_linear_speed;
        double pidout_angular_speed = exec_pwm_gain * exec_angular_speed;
        double pidout_direction     = exec_desired_direction;
        this->motor_handler->execute(pidout_linear_speed,pidout_direction,pidout_angular_speed);
    }
    else
    {
        printf ("ERROR! unknown control mode \n");
        exec_linear_speed = 0;
        exec_angular_speed = 0;
        exec_pwm_gain = 0;
        exec_desired_direction = 0;
        this->motor_handler->execute(0,0,0);
    }
}
