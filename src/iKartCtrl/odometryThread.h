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

#ifndef ODOMETRY_THREAD_H
#define ODOMETRY_THREAD_H

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
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define M_PI 3.14159265

class OdometryThread: public yarp::os::RateThread
{
private:
	Property iKartCtrl_options;
    yarp::os::Semaphore   mutex;
 
	//encoder variables
    double              encA_offset;
	double              encB_offset;
	double              encC_offset;

	double              encA;
	double              encB;
	double              encC;

    //estimated motor velocity
    double              velA;
	double              velB;
	double              velC;

    //estimated cartesian velocity
    double              vel_lin;
	double              vel_ang;

    //estimated cartesian pos
    double              odom_x;
	double              odom_y;
    double              odom_theta;

    //robot geometry
    double              geom_r;
    double              geom_L;

protected:
    ResourceFinder                  &rf;
	PolyDriver                      *control_board_driver;
    BufferedPort<Bottle>            port_a;
	BufferedPort<Bottle>            port_b;

    string remoteName;
    string localName;

	IEncoders         *ienc;

public:
    OdometryThread(unsigned int _period, ResourceFinder &_rf, Property options,
               PolyDriver* _driver) :
               RateThread(_period),     rf(_rf),
			   iKartCtrl_options (options)
	{
               control_board_driver= _driver;
               odom_x=0;
	           odom_y=0;
               odom_theta=0;		
               geom_r = 62.5;     //mm
               geom_L = 297.16;   //mm
    }

	
    virtual bool threadInit()
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
        port_a.open((localName+"/odom/a").c_str());
		port_b.open((localName+"/odom/b").c_str());

        ienc->getEncoder(0,&encA_offset);
        ienc->getEncoder(1,&encB_offset);
        ienc->getEncoder(2,&encC_offset);

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Odometry thread started successfully\n");
        else
            printf("Odometry thread did not start\n");
    }

    virtual void run()
    {
        mutex.wait();

        ienc->getEncoder(0,&encA);
        ienc->getEncoder(1,&encB);
        ienc->getEncoder(2,&encC);
        //remove the offset and convert in radians
        encA= (encA - encA_offset) * 0.0174532925; 
        encB= (encB - encB_offset) * 0.0174532925;
        encC= (encC - encC_offset) * 0.0174532925;
       
        ienc->getEncoderSpeed(0,&velA);
        ienc->getEncoderSpeed(1,&velB);
        ienc->getEncoderSpeed(2,&velC);

        // -------------------------------------------------------------------------------------
        // The following formulas are adapted from:
        // "A New Odometry System to reduce asymmetric Errors for Omnidirectional Mobile Robots"
        // -------------------------------------------------------------------------------------

        //odom_theta is expressed in radians
        odom_theta = geom_r*(encA+encB+encC)/(3*geom_L);

        double co3p = cos (M_PI/3+odom_theta);
        double si3p = cos (M_PI/3+odom_theta);
        double co3m = cos (M_PI/3-odom_theta);
        double si3m = cos (M_PI/3-odom_theta);

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

        //convert from radians back to degrees
        odom_theta *= 57.2957795;
        encA       *= 57.2957795;
        encB       *= 57.2957795;
        encC       *= 57.2957795;

        mutex.post();

		Bottle &b=port_a.prepare();
        b.addDouble(encA);
        b.addDouble(encB);
        b.addDouble(encC);
        b.addDouble(velA);
        b.addDouble(velB);
        b.addDouble(velC);
        b.addDouble(odom_x);
        b.addDouble(odom_y);
        b.addDouble(odom_theta);
		port_a.write();
    }

    virtual void threadRelease()
    {    
        port_a.interrupt();
        port_a.close();
		port_b.interrupt();
        port_b.close();
    }

    void printStats()
    {
        mutex.wait();
		//fprintf (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
        fprintf (stdout,"Odometry Thread: \
        %+3.3f %+3.3f %+3.3f ******** \
        %+3.3f %+3.3f %+3.3f ******** \
        %+3.3f %+3.3f %+3.3f \
        \n", encA, encB, encC, velA, velB, velC, odom_x, odom_y, odom_theta);
        mutex.post();
    }
};

#endif
