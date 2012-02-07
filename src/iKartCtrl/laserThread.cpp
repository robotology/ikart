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

#include "laserThread.h"

bool LaserThread::threadInit()
{
    ConstString laser_filename = iKartCtrl_options.findGroup("GENERAL").find("laser").asString();
    ConstString laser_config_filename =rf.findFile(laser_filename);        
    if (laser_config_filename=="") 
    {
        printf("\nError! Unable to locate .ini laser configuration file. \nLooking for %s\n",laser_config_filename.c_str());
        return false;
    }
    else
    {
        printf("\nOpening the laser interface...\n");
        Property laser_options;
        laser_options.fromConfigFile(laser_config_filename.c_str());
        laser_options.put("CONFIG_PATH",rf.getContextPath().c_str());

        // open the laser scanner driver
        laser_driver=new PolyDriver;
        laser_options.put("device","laserHokuyo");
        if (fake_laser) laser_options.put("fake","");
        if (!laser_driver->open(laser_options))
        {
            fprintf(stderr,"ERROR: cannot open laser driver...\n");
            delete laser_driver;    
            return false;
        }
        //open the interface for the laser
        bool laser_ok = laser_driver->view(iLaser);
        if(!laser_ok)
        {
            fprintf(stderr,"ERROR: cannot view the laser interface\nreturning...\n");
            return false;
        }
        // open the laser output port
        port_laser_output.open((localName+"/laser:o").c_str());
    }

    return true;
}

void LaserThread::run()
{
    yarp::sig::Vector laser_data;
    //fprintf(stderr,"before laser reading\n");
    double before_laser=Time::now();
    int res = iLaser->read(laser_data);
    double after_laser=Time::now();
    if (after_laser-before_laser > 0.040) { timeout_counter++;  }
    //fprintf(stderr,"after laser reading\n");
    if (res == yarp::dev::IAnalogSensor::AS_OK && port_laser_output.getOutputCount()>0)
    {
        yarp::sig::Vector &plaser_data=port_laser_output.prepare();
        plaser_data=laser_data;
        //lastStateStamp.update();
        //port_laser_data.setEnvelope(lastStateStamp);
        port_laser_output.write();
    }
    else
    {
        fprintf(stderr,"Error reading laser data, code: %d\n", res);
    }
}

void LaserThread::printStats()
{
    fprintf (stdout,"Laser thread timeouts: %d\n",timeout_counter);
}
