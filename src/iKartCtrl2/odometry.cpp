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

#include "odometry.h"

bool Odometry::reset_odometry()
{
    ienc->getEncoder(0,&encA_offset);
    ienc->getEncoder(1,&encB_offset);
    ienc->getEncoder(2,&encC_offset);
    fprintf(stderr,"Odometry reset done\n");
    return true;
}

void Odometry::printStats()
{
    mutex.wait();
    //fprintf (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
    fprintf (stdout,"Odometry Thread: \
    %+3.3f %+3.3f %+3.3f ******** \
    %+3.3f %+3.3f %+3.3f ******** \
    vx:%+3.3f vy:%+3.3f vt:%+3.3f ******** \
    x: %+3.3f y: %+3.3f t: %+3.3f ******** \
    \n", enc[0]*57, enc[1]*57, enc[2]*57, encv[0]*57, encv[1]*57, encv[2]*57, vel_x, vel_y, vel_theta,  odom_x, odom_y,odom_theta );
    mutex.post();
}

void Odometry::close()
{    
    port_odometry.interrupt();
    port_odometry.close();
    port_odometer.interrupt();
    port_odometer.close();
}

Odometry::~Odometry()
{
    close();
}

Odometry::Odometry(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver):rf(_rf)
{
    iKartCtrl_options = options;
    period = _period;
    control_board_driver= _driver;
    odom_x=0;
    odom_y=0;
    odom_theta=0;
    vel_x=0;
    vel_y=0;
    vel_lin=0;
    vel_theta=0;
    vel_heading=0;
    traveled_distance=0;
    traveled_angle=0;
    geom_r = 62.5/1000.0;     //m
    geom_L = 297.16/1000.0;   //m
    encvel_estimator =new iCub::ctrl::AWLinEstimator(3,1.0);
    enc.resize(3);
    encv.resize(3);
    localName = iKartCtrl_options.find("local").asString();
}

bool Odometry::open()
{
    // open the control board driver
    printf("\nOpening the motors interface...\n");

    Property control_board_options("(device remote_controlboard)");
    if (!control_board_driver)
    {
        fprintf(stderr,"ERROR: control board driver not ready!\n");
            return false;
    }
    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ienc);
    if(!ok)
    {
        fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...\n");
        //return false;
    }
    // open control input ports
    port_odometry.open((localName+"/odometry:o").c_str());
    port_odometer.open((localName+"/odometer:o").c_str());

    //reset odometry
    reset_odometry();

    return true;
}

void Odometry::compute()
{
    mutex.wait();

    //read the encoders (deg)
    ienc->getEncoder(0,&encA);
    ienc->getEncoder(1,&encB);
    ienc->getEncoder(2,&encC);
        
    //read the speeds (deg/s)
    ienc->getEncoderSpeed(0,&velA);
    ienc->getEncoderSpeed(1,&velB);
    ienc->getEncoderSpeed(2,&velC);
        
    //remove the offset and convert in radians
    enc[0]= -(encA - encA_offset) * 0.0174532925; 
    enc[1]= -(encB - encB_offset) * 0.0174532925;
    enc[2]= -(encC - encC_offset) * 0.0174532925;
       
    //estimate the speeds
    iCub::ctrl::AWPolyElement el;
    el.data=enc;
    el.time=Time::now();
    encv= encvel_estimator->estimate(el);   

    // -------------------------------------------------------------------------------------
    // The following formulas are adapted from:
    // "A New Odometry System to reduce asymmetric Errors for Omnidirectional Mobile Robots"
    // -------------------------------------------------------------------------------------

    //compute the orientation. odom_theta is expressed in radians
    odom_theta = geom_r*(enc[0]+enc[1]+enc[2])/(3*geom_L);

    //build the kinematics matrix
    yarp::sig::Matrix kin;
    kin.resize(3,3);
    kin.zero();
    kin(0,0) = -sqrt(3.0)/2;
    kin(0,1) = 0.5;
    kin(0,2) = geom_L;
    kin(1,0) = sqrt(3.0)/2;
    kin(1,1) = 0.5;
    kin(1,2) = geom_L;
    kin(2,0) = 0;
    kin(2,1) = -1;
    kin(2,2) = geom_L;
    yarp::sig::Matrix ikin = luinv(kin);

    //build the rotation matrix
    yarp::sig::Matrix m;
    m.resize(3,3);
    m.zero();        
    m(0,0) = cos (odom_theta);
    m(0,1) = -sin (odom_theta);
    m(1,0) = sin (odom_theta);
    m(1,1) = cos (odom_theta);
    m(2,2) = 1;

    yarp::sig::Vector cart_vels;
    cart_vels = m*ikin*encv;
    vel_x     = cart_vels[1]*geom_r;
    vel_y     = cart_vels[0]*geom_r;
    vel_lin   = vel_x*vel_x + vel_y*vel_y;
    if (vel_lin<0.001) vel_heading= 0;
    else  vel_heading = atan2(vel_x,vel_y)*57.2957795;
    vel_theta = cart_vels[2];

    /*double co3p = cos (M_PI/3+odom_theta);
    double si3p = cos (M_PI/3+odom_theta);
    double co3m = cos (M_PI/3-odom_theta);
    double si3m = cos (M_PI/3-odom_theta);*/
    
    //the integration step
    odom_x=odom_x + (vel_x * period/1000.0);
    odom_y=odom_y + (vel_y * period/1000.0);

    //compute traveled distance (odometer)
    traveled_distance = traveled_distance + fabs(vel_lin * period/1000.0);
    traveled_angle = traveled_angle + fabs(vel_theta * period/1000.0);

    /* [ -(3^(1/2)*r)/3, (3^(1/2)*r)/3,        0]
        [            r/3,           r/3, -(2*r)/3]
        [        r/(3*L),       r/(3*L),  r/(3*L)]*/

    /*odom_x = -1.73205080*geom_r/3 * encA + 1.73205080*geom_r/3 * encB;
    odom_y = geom_r/3 * encA +  geom_r/3 * encB - (2*geom_r)/3 * encC;*/
        
    /*
    odom_x = geom_r/(3* 0.86602)*
                (
                (co3p-co3m)*encA + 
                (-cos(odom_theta)-co3p)*encB + 
                (cos(odom_theta)+co3m)*encC
                );
    odom_y = geom_r/(3* 0.86602)*
                (
                (si3m+si3p)*encA + 
                (-sin(odom_theta)-si3p)*encB + 
                (sin(odom_theta)-si3m)*encC
                );
    */

    //convert from radians back to degrees
    odom_theta *= 57.2957795;

    mutex.post();

	Bottle &b=port_odometry.prepare();
    b.clear();
    b.addDouble(odom_x);
    b.addDouble(odom_y);
    b.addDouble(odom_theta);
    b.addDouble(vel_x);
    b.addDouble(vel_y);
    b.addDouble(vel_theta);
    port_odometry.write();

    Bottle &t=port_odometer.prepare();
    t.clear();
    t.addDouble(traveled_distance);
    t.addDouble(traveled_angle);
    port_odometer.write();
}
