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

#include "motors.h"

int MotorControl::get_ikart_control_type()
{
    return ikart_control_type;
}

void MotorControl::set_ikart_control_type(int type)
{
    ikart_control_type = type;

    if (ikart_control_type == IKART_CONTROL_OPENLOOP)
    {
        fprintf(stdout,"iKart in openloop control mode\n");
        icmd->setOpenLoopMode(0);
        icmd->setOpenLoopMode(1);
        icmd->setOpenLoopMode(2);
        iopl->setOutput(0,0);
        iopl->setOutput(1,0);
        iopl->setOutput(2,0);
    }
    else if (ikart_control_type == IKART_CONTROL_SPEED)
    {
        fprintf(stdout,"iKart in speed control mode\n");
        icmd->setVelocityMode(0);
        icmd->setVelocityMode(1);
        icmd->setVelocityMode(2);
    }
    else
    {
        fprintf(stdout,"invalid iKart control mode\n");
    }
}

double MotorControl::lp_filter_1Hz(double input, int i)
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
double MotorControl::lp_filter_0_5Hz(double input, int i)
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

double MotorControl::ratelim_filter_0(double input, int i)
{
    //This is a rate limiter filter. 
    static double prev[3];
    if      (input>prev[i]+10) prev[i]=prev[i]+10;
    else if (input<prev[i]-10) prev[i]=prev[i]-10;
    else     prev[i]=input;
    return prev[i];
}

bool MotorControl::turn_off_control()
{
    iamp->disableAmp(0);
    iamp->disableAmp(1);
    iamp->disableAmp(2);
    set_ikart_control_type (IKART_CONTROL_NONE);
    return true;
}

bool MotorControl::turn_on_speed_control()
{
    iamp->enableAmp(0);
    iamp->enableAmp(1);
    iamp->enableAmp(2);
    set_ikart_control_type(IKART_CONTROL_SPEED);
    int c0(0),c1(0),c2(0);
    yarp::os::Time::delay(0.05);
    icmd->getControlMode(0,&c0);
    icmd->getControlMode(0,&c1);
    icmd->getControlMode(0,&c2);
    if (c0==VOCAB_CM_VELOCITY && 
        c1==VOCAB_CM_VELOCITY && 
        c2==VOCAB_CM_VELOCITY) 
        return true;
    else
        return false;
}

void MotorControl::updateControlMode()
{
    icmd->getControlModes(board_control_modes);
    /*
        for (int i=0; i<3; i++)
        if (board_control_modes[i]==VOCAB_CM_IDLE)
        {
            fprintf (stderr,"One motor is in idle state. Turning off control.");
            turn_off_control();
            break;
        }
    */
}

void MotorControl::printStats()
{
    fprintf (stdout,"Motor thread timeouts: %d joy: %d cmd %d\n",thread_timeout_counter, joy_timeout_counter,mov_timeout_counter);

    if (joystick_received>0) 
        fprintf (stdout,"Under joystick control (%d)\n",joystick_received);

    double val = 0;
    for (int i=0; i<3; i++)
    {
        if      (i==0) val=FA;
        else if (i==1) val=FB;
        else           val=FC;
        if (board_control_modes[i]==VOCAB_CM_IDLE)
            fprintf (stdout,"F%d: IDLE\n",i);
        else
            fprintf (stdout,"F%d: %+.1f\n",i,val);	
    }

    fprintf (stdout,"\n");
}

void MotorControl::close()
{
    port_movement_control.interrupt();
    port_movement_control.close();
    port_auxiliary_control.interrupt();
    port_auxiliary_control.close();
    port_joystick_control.interrupt();
    port_joystick_control.close();
}

MotorControl::~MotorControl()
{
    close();
}

bool MotorControl::open()
{
    ConstString control_type = iKartCtrl_options.check("control_mode",Value("none"),"type of control for the wheels").asString();
    if      (control_type == "none")     ikart_control_type = IKART_CONTROL_NONE;
    else if (control_type == "speed")    ikart_control_type = IKART_CONTROL_SPEED;
    else if (control_type == "openloop") ikart_control_type = IKART_CONTROL_OPENLOOP;
    else
    {
        fprintf(stderr,"Error: unknown type of control required: %s. Closing...\n",control_type.c_str());
        return false;
    }

    if (rf.check("no_filter"))
    {
        printf("\n'no_filter' option found. Turning off PWM filter.\n");
        filter_enabled=false;
    }

    lin_ang_ratio = iKartCtrl_options.check("linear_angular_ratio",Value(0.7),"ratio (<1.0) between the maximum linear speed and the maximum angular speed.").asDouble();

    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ivel);
    ok = ok & control_board_driver->view(ienc);
    ok = ok & control_board_driver->view(iopl);
    ok = ok & control_board_driver->view(ipid);
    ok = ok & control_board_driver->view(iamp);
    ok = ok & control_board_driver->view(icmd);
    if(!ok)
    {
        fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...\n");
        //return false;
    }
    // open control input ports
    port_movement_control.open((localName+"/control:i").c_str());
    port_auxiliary_control.open((localName+"/aux_control:i").c_str());
    port_joystick_control.open((localName+"/joystick:i").c_str());

    //set the control type
    if (!rf.check("no_start"))
    {
        printf("starting motors...");
        iamp->enableAmp(0);
        iamp->enableAmp(1);
        iamp->enableAmp(2);
    }
    set_ikart_control_type(IKART_CONTROL_SPEED);

    return true;
}

MotorControl::MotorControl(unsigned int _period, ResourceFinder &_rf, Property options,
            PolyDriver* _driver) :
            rf(_rf),
            iKartCtrl_options (options)
    {
    control_board_driver= _driver;
    ikart_control_type  = IKART_CONTROL_NONE;

    thread_timeout_counter     = 0;

    command_received    = 0;
    joystick_received   = 0;
    auxiliary_received  = 0;

    mov_timeout_counter = 0;
    joy_timeout_counter = 0;
    aux_timeout_counter = 0;

    FA = 0;
    FB = 0;
    FC = 0;

    filter_enabled = true;
    lin_ang_ratio = 0.7;
    both_lin_ang_enabled = true;
    thread_period = _period;
    localName = iKartCtrl_options.find("local").asString();
}

void MotorControl::read_inputs(double *linear_speed,double *angular_speed,double *desired_direction, double *pwm_gain)
{
    static double wdt_old_mov_cmd=Time::now();
    static double wdt_old_joy_cmd=Time::now();
    static double wdt_old_aux_cmd=Time::now();
    static double wdt_mov_cmd=Time::now();
    static double wdt_joy_cmd=Time::now();
    static double wdt_aux_cmd=Time::now();

    if (Bottle *b = port_joystick_control.read(false))
    {                
        if (b->get(0).asInt()==1)
        {                                
            //received a joystick command.
            joy_desired_direction = b->get(1).asDouble();
            joy_linear_speed      = b->get(2).asDouble();
            joy_angular_speed     = b->get(3).asDouble();
            joy_pwm_gain          = b->get(4).asDouble();
            joy_linear_speed = (joy_linear_speed<+100) ? joy_linear_speed : +100; 
            joy_linear_speed = (joy_linear_speed>-100) ? joy_linear_speed : -100;
            joy_angular_speed = (joy_angular_speed<+100) ? joy_angular_speed : +100; 
            joy_angular_speed = (joy_angular_speed>-100) ? joy_angular_speed : -100;
            joy_pwm_gain = (joy_pwm_gain<+100) ? joy_pwm_gain : +100; 
            joy_pwm_gain = (joy_pwm_gain>0) ? joy_pwm_gain : 0;
            wdt_old_joy_cmd = wdt_joy_cmd;
            wdt_joy_cmd = Time::now();

            //Joystick commands have higher priorty respect to movement commands.
            //this make the joystick to take control for 100*20 ms
            if (joy_pwm_gain>10) joystick_received = 100;
        }
    }
    if (Bottle *b = port_movement_control.read(false))
    {
        if (b->get(0).asInt()==1)
        {
            cmd_desired_direction = b->get(1).asDouble();
            cmd_linear_speed      = b->get(2).asDouble();
            cmd_angular_speed     = b->get(3).asDouble();
            cmd_pwm_gain          = b->get(4).asDouble();
            cmd_linear_speed = (cmd_linear_speed<+100) ? cmd_linear_speed : +100; 
            cmd_linear_speed = (cmd_linear_speed>-100) ? cmd_linear_speed : -100;
            cmd_angular_speed = (cmd_angular_speed<+100) ? cmd_angular_speed : +100; 
            cmd_angular_speed = (cmd_angular_speed>-100) ? cmd_angular_speed : -100;
            cmd_pwm_gain = (cmd_pwm_gain<+100) ? cmd_pwm_gain : +100; 
            cmd_pwm_gain = (cmd_pwm_gain>0) ? cmd_pwm_gain : 0;
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else if (b->get(0).asInt()==2)
        {
            double x_speed        = b->get(1).asDouble() * 100.0 / MAX_LINEAR_VEL;
            double y_speed        = b->get(2).asDouble() * 100.0 / MAX_LINEAR_VEL;
            double t_speed        = b->get(3).asDouble() * 100.0 / MAX_ANGULAR_VEL;
            //t_speed = (t_speed<+100) ? t_speed : +100; 
            //t_speed = (t_speed>-100) ? t_speed : -100;
            cmd_desired_direction = atan2(x_speed,y_speed) * 180.0 / 3.14159265;
            cmd_linear_speed      = sqrt (x_speed*x_speed+y_speed*y_speed);
            cmd_angular_speed     = t_speed;
            cmd_pwm_gain          = 100;
            cmd_linear_speed = (cmd_linear_speed<+100) ? cmd_linear_speed : +100; 
            cmd_linear_speed = (cmd_linear_speed>-100) ? cmd_linear_speed : -100;
            cmd_angular_speed = (cmd_angular_speed<+100) ? cmd_angular_speed : +100; 
            cmd_angular_speed = (cmd_angular_speed>-100) ? cmd_angular_speed : -100;
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
    }
    if (Bottle *b = port_auxiliary_control.read(false))
    {
        if (b->get(0).asInt()==1)
        {
            aux_desired_direction = b->get(1).asDouble();
            aux_linear_speed      = b->get(2).asDouble();
            aux_angular_speed     = b->get(3).asDouble();
            aux_pwm_gain          = b->get(4).asDouble();
            aux_linear_speed = (aux_linear_speed<+100) ? aux_linear_speed : +100; 
            aux_linear_speed = (aux_linear_speed>-100) ? aux_linear_speed : -100;
            aux_angular_speed = (aux_angular_speed<+100) ? aux_angular_speed : +100; 
            aux_angular_speed = (aux_angular_speed>-100) ? aux_angular_speed : -100;
            aux_pwm_gain = (aux_pwm_gain<+100) ? aux_pwm_gain : +100; 
            aux_pwm_gain = (aux_pwm_gain>0) ? aux_pwm_gain : 0;
            wdt_old_aux_cmd = wdt_aux_cmd;
            wdt_aux_cmd = Time::now();
            auxiliary_received = 100;
        }
    }

    //priority test 
    if (joystick_received>0)
    {
        *desired_direction  = joy_desired_direction;
        *linear_speed       = joy_linear_speed;
        *angular_speed      = joy_angular_speed;
        *pwm_gain           = joy_pwm_gain;
    }
    else if (command_received>0)
    {
        *desired_direction  = cmd_desired_direction;
        *linear_speed       = cmd_linear_speed;
        *angular_speed      = cmd_angular_speed;
        *pwm_gain           = cmd_pwm_gain;
    }
    else
    {
        *desired_direction  = aux_desired_direction;
        *linear_speed       = aux_linear_speed;
        *angular_speed      = aux_angular_speed;
        *pwm_gain           = aux_pwm_gain;
    }

    //watchdog on received commands
    static double wdt_old=Time::now();
    double wdt=Time::now();
    //fprintf(stderr,"period: %f\n", wdt-wdt_old);
    if (wdt-wdt_mov_cmd > 0.200)
    {
        cmd_desired_direction=0;
        cmd_linear_speed=0;
        cmd_angular_speed=0;
        cmd_pwm_gain=0;
        mov_timeout_counter++; 
    }
    if (wdt-wdt_joy_cmd > 0.200)
    {
        joy_desired_direction=0;
        joy_linear_speed=0;
        joy_angular_speed=0;
        joy_pwm_gain=0;
        joy_timeout_counter++;
    }
    if (wdt-wdt_aux_cmd > 0.200)
    {
        aux_desired_direction=0;
        aux_linear_speed=0;
        aux_angular_speed=0;
        aux_pwm_gain=0;
        aux_timeout_counter++;
    }

    if (wdt-wdt_old > 0.040) { thread_timeout_counter++;  }
    wdt_old=wdt;

    if (joystick_received>0)   { joystick_received--; }
    if (command_received>0)    { command_received--; }
    if (auxiliary_received>0)  { auxiliary_received--; }
}

void MotorControl::execute(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    //wheel contribution calculation
    FA = appl_linear_speed * cos ((150.0-appl_desired_direction)/ 180.0 * 3.14159265) + appl_angular_speed;
    FB = appl_linear_speed * cos ((030.0-appl_desired_direction)/ 180.0 * 3.14159265) + appl_angular_speed;
    FC = appl_linear_speed * cos ((270.0-appl_desired_direction)/ 180.0 * 3.14159265) + appl_angular_speed;
        
    //Use a low pass filter to obtain smooth control
    if (filter_enabled)
    {
        //FA  = lp_filter_0_5Hz(FA,0);
        //FB  = lp_filter_0_5Hz(FB,1);
        //FC  = lp_filter_0_5Hz(FC,2);
        //FA  = ratelim_filter_0(FA,0);
        //FB  = ratelim_filter_0(FB,1);
        //FC  = ratelim_filter_0(FC,2);
    }

    //Apply the commands
    if (ikart_control_type == IKART_CONTROL_OPENLOOP)
    {
        iopl->setOutput(0,-FA);
        iopl->setOutput(1,-FB);
        iopl->setOutput(2,-FC);
    }
    else if	(ikart_control_type == IKART_CONTROL_SPEED)
    {
//#define CONTROL_DEBUG
#ifdef  CONTROL_DEBUG
        fprintf (stdout,">**: %+6.6f %+6.6f **** %+6.6f %+6.6f %+6.6f\n",exec_linear_speed,exec_desired_direction,-FA,-FB,-FC);
#endif
        ivel->velocityMove(0,-FA);
        ivel->velocityMove(1,-FB);
        ivel->velocityMove(2,-FC);
    }
    else if (ikart_control_type == IKART_CONTROL_NONE)
    {
        //iopl->setOutput(0,0);
        //iopl->setOutput(1,0);
        //iopl->setOutput(2,0);
    }
}
