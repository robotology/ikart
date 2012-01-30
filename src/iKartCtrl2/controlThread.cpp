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

void ControlThread::apply_ratio_limiter (double max, double& linear_speed, double& angular_speed)
{
    if (lin_ang_ratio<0.0)  lin_ang_ratio = 0.0;
    if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
    if (linear_speed  >  max*lin_ang_ratio) linear_speed  = max*lin_ang_ratio;
    if (linear_speed  < -max*lin_ang_ratio) linear_speed  = -max*lin_ang_ratio;
    if (angular_speed >  max*(1-lin_ang_ratio)) angular_speed = max*(1-lin_ang_ratio);
    if (angular_speed < -max*(1-lin_ang_ratio)) angular_speed = -max*(1-lin_ang_ratio);
}

void ControlThread::apply_pre_filter (double& linear_speed, double& angular_speed)
{
    if (pre_filter_enabled)
    {
        angular_speed  = ikart_filters::lp_filter_8Hz(angular_speed,7);
        linear_speed   = ikart_filters::lp_filter_8Hz(linear_speed,8);
    }
}

void ControlThread::set_pid (string id, double kp, double ki, double kd)
{
    yarp::os::Bottle old_options;
    this->angular_speed_pid->getOptions(old_options);
    printf("Current configuration: %s\n",old_options.toString().c_str());
    
    // (Kp (10.0)) (Ki (0.0)) (Kf (0.0)) ... (satLim(-1000.0 1000.0)) (Ts 0.02)
    yarp::os::Bottle options;
    yarp::os::Bottle& bkp = options.addList();
    yarp::os::Bottle& bki = options.addList();
    yarp::os::Bottle& bkd = options.addList();
    bkp.addString("Kp");    yarp::os::Bottle& bkp2 = bkp.addList();    bkp2.addDouble(kp);
    bki.addString("Ki");    yarp::os::Bottle& bki2 = bki.addList();    bki2.addDouble(ki);
    bkd.addString("Kd");    yarp::os::Bottle& bkd2 = bkd.addList();    bkd2.addDouble(kd);
    printf("new configuration: %s\n",options.toString().c_str());

    this->angular_speed_pid->setOptions(options);
    yarp::sig::Vector tmp; tmp.resize(1); tmp.zero();
    this->angular_speed_pid->reset(tmp);
}

void ControlThread::run()
{
    this->odometry_handler->compute();

    double              linear_speed=0;
    double              angular_speed=0;
    double              desired_direction=0;

    double              pwm_gain=0;
    this->motor_handler->read_inputs(&linear_speed, &angular_speed, &desired_direction, &pwm_gain);
    if (!lateral_movement_enabled) 
    {
        if (desired_direction>-90 && desired_direction <90) desired_direction = 0;
        else if (desired_direction <= -90) desired_direction = 180;
        else if (desired_direction >= +90) desired_direction = 180;
    }

    //Here we suppouse that values are already in the format 0-100%
    double MAX_VALUE = 0;
    if (motor_handler->get_ikart_control_type() == IKART_CONTROL_OPENLOOP_NO_PID)
    {
        MAX_VALUE = 1333; // Maximum joint PWM
        exec_linear_speed  = linear_speed  / 100.0 * 1300.0;
        exec_angular_speed = angular_speed / 100.0 * 1300.0;
        exec_pwm_gain = pwm_gain / 100.0 * 1.0;
        exec_desired_direction = desired_direction;
        apply_ratio_limiter(MAX_VALUE, exec_linear_speed, exec_angular_speed);
        apply_pre_filter(exec_linear_speed, exec_angular_speed);
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
        //exec_linear_speed = linear_speed / 100.0 * 0.1/*MAX_VALUE*/;  //0.1 m/s
        //exec_angular_speed = angular_speed / 100.0 * 20/*MAX_VALUE*/; //20 deg/s
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
        exec_linear_speed = linear_speed / 100.0 * MAX_VALUE;
        exec_angular_speed = angular_speed / 100.0 * MAX_VALUE;
        exec_pwm_gain = pwm_gain / 100.0 * 1.0;
        exec_desired_direction = desired_direction;
        apply_ratio_limiter(MAX_VALUE, exec_linear_speed, exec_angular_speed);
        apply_pre_filter(exec_linear_speed, exec_angular_speed);
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
