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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

#include "ikartGoto.h"
#include <math.h>

class iKartGotoModule : public yarp::os::RFModule
{
protected:
    GotoThread     *gotoThread;
    yarp::os::Port rpcPort;

public:
    iKartGotoModule()
    {
        gotoThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        Property p;
        ConstString configFile = rf.findFile("from");
        if (configFile!="") p.fromConfigFile(configFile.c_str());

        gotoThread = new GotoThread(10,rf,p);
        if (rf.check("no_stop_on_obstacles")) gotoThread->enable_stop_on_obstacles=false;

        if (!gotoThread->start())
        {
            delete gotoThread;
            return false;
        }

        rpcPort.open("/ikartGoto/rpc:i");
        attach(rpcPort);
        //attachTerminal();

        return true;
    }

    virtual bool interruptModule()
    {
        rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        //gotoThread->shutdown();
        gotoThread->stop();
        delete gotoThread;
        gotoThread=NULL;

        return true;
    }

    virtual double getPeriod()
    { 
        if (gotoThread) gotoThread->printStats();
        return 1.0; 
    }
    
    virtual bool updateModule()
    { 
        if (isStopping())
        {
            gotoThread->stop();   
            return false;
        }
        
        return true; 
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        reply.clear(); 

        if (command.get(0).asString()=="quit") return false;     

        else if (command.get(0).asString()=="help")
        {
            reply.addString("Available commands are:");
            reply.addString("goToAbs <x> <y> <angle>");
            reply.addString("goToRel <x> <y> <angle>");
            reply.addString("stop");
            reply.addString("pause");
            reply.addString("resume");
            reply.addString("quit");
            reply.addString("set linear_tol <m>");
            reply.addString("set linear_ang <deg>");
        }

        else if (command.get(0).asString()=="gotoAbs")
        {
            yarp::sig::Vector v;
            v.push_back(command.get(1).asDouble());
            v.push_back(command.get(2).asDouble());
            if (command.size()==4) v.push_back(command.get(3).asDouble());
            gotoThread->setNewAbsTarget(v);
            reply.addString("new absolute target received");
        }

        else if (command.get(0).asString()=="gotoRel")
        {
            yarp::sig::Vector v;
            v.push_back(command.get(1).asDouble());
            v.push_back(command.get(2).asDouble());
            if (command.size()==4) v.push_back(command.get(3).asDouble());
            gotoThread->setNewRelTarget(v);
            reply.addString("new relative target received");
        }
        else if (command.get(0).asString()=="set")
        {
            if (command.get(1).asString()=="linear_tol")
            {
                gotoThread->goal_tolerance_lin=command.get(2).asDouble();
                reply.addString("linear_tol set.");
            }
            else if (command.get(1).asString()=="angular_tol")
            {
                gotoThread->goal_tolerance_ang=command.get(2).asDouble();
                reply.addString("angular_tol set.");
            }
            else
            {
                reply.addString("Unknown set.");
            }
        }
        else if (command.get(0).asString()=="get")
        {
            if (command.get(1).asString()=="navigation_status")
            {
                string s = gotoThread->getNavigationStatus();
                reply.addString(s.c_str());
            }
            else
            {
                reply.addString("Unknown get.");
            }
        }
        else if (command.get(0).asString()=="stop")
        {
            gotoThread->stopMovement();
            reply.addString("Stopping movement.");
        }
        else if (command.get(0).asString()=="pause")
        {
            double time = -1;
            if (command.size() > 1)
                time = command.get(1).asDouble();
            gotoThread->pauseMovement(time);
            reply.addString("Pausing.");
        }
        else if (command.get(0).asString()=="resume")
        {
            gotoThread->resumeMovement();
            reply.addString("Resuming.");
        }
        else
        {
            reply.addString("Unknown command.");
        }

        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"ERROR: check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("iKartGoto.ini");           //overridden by --from parameter
    rf.setDefaultContext("iKartGoto/conf");                //overridden by --context parameter
    rf.configure("ICUB_ROOT",argc,argv);
    
    iKartGotoModule iKartGoto;

    return iKartGoto.runModule(rf);
}

 
