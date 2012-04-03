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

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

#define WAIT_TIME 3.0

class iKartUtilsModule: public RFModule
{
    enum command_type {NO_CMD=0, CLOSE = 0, OPEN =1};
    PolyDriver       *left_arm_device;
    PolyDriver       *right_arm_device;
    IPositionControl *l_pos ;
    IEncoders        *l_encs;
    IPositionControl *r_pos ;
    IEncoders        *r_encs;
    bool             running;
    Port                  rpcPort;
    BufferedPort<Bottle>  port_ikart_ctrl;

    int                   approach_count;
    int                   retreat_count;

public:
    iKartUtilsModule()
    {
        approach_count   = 0;
        retreat_count    = 0;
        left_arm_device  = 0;
        right_arm_device = 0;
        l_pos  = 0;
        l_encs = 0;
        r_pos  = 0;
        r_encs = 0;
        running = true;
    }

    void approach(int dur)
    {
        approach_count = dur;
    }
    
    void retreat(int dur)
    {
        retreat_count = dur;
    }

    void tuck(int tuck_cmd)
    {
        Vector command;
        command.resize  (16,0.0);
        if (tuck_cmd==OPEN)
        {
            command[0]=-30;
            command[1]=30;
            command[2]=-0;
            command[3]=45;
        }
        else if (tuck_cmd==CLOSE)
        {
            command[0]=10;
            command[1]=15;
            command[2]=0;
            command[3]=25;
        }
        if (l_pos) l_pos->positionMove(command.data());
        if (r_pos) r_pos->positionMove(command.data());
    }

    virtual bool configure(ResourceFinder &rf)
    {
        rpcPort.open("/iKartUtils/rpc");
        port_ikart_ctrl.open("/iKartUtils/ikart_commands:o");
        yarp::os::Network::connect("/iKartUtils/ikart_commands:o", "/ikart/aux_control:i");
        attach(rpcPort);

        std::string robotName = rf.check("robot",yarp::os::Value("icub")).asString().c_str();

        command_type tuck_cmd = NO_CMD;
        if      (rf.check("close")) tuck_cmd = CLOSE;
        else if (rf.check("open"))  tuck_cmd = OPEN;

        std::string localPorts;
        std::string remotePorts;

        Property left_options;
        left_options.put("device", "remote_controlboard");
        remotePorts="/"+robotName+"/left_arm";
        localPorts="/iKartUtils/left_arm";
        left_options.put("local",  localPorts.c_str());
        left_options.put("remote", remotePorts.c_str());

        Property right_options;
        right_options.put("device", "remote_controlboard");
        remotePorts="/"+robotName+"/right_arm";
        localPorts="/iKartUtils/right_arm";
        right_options.put("local",  localPorts.c_str());
        right_options.put("remote", remotePorts.c_str());

        // create the devices
        left_arm_device  = new PolyDriver (left_options);
        right_arm_device = new PolyDriver (right_options);
        if (!left_arm_device->isValid())
        {
            printf("Error opening the left arm device\n");
        }
        if (!right_arm_device->isValid())
        {
            printf("Error opening the right arm device\n");
        }

        //crate the interfaces
        bool ok = true;
        if (left_arm_device)
        {
            ok &= left_arm_device->view (l_pos);
            ok &= left_arm_device->view (l_encs);
        }
        if (right_arm_device)
        {
            ok &= right_arm_device->view(r_pos);
            ok &= right_arm_device->view(r_encs);
        }

        if (!ok)
        {
            printf("Problems acquiring interfaces\n");
            return 0;
        }

        const int nj=15;
        Vector encoders;
        Vector command;
        Vector accs;
        Vector spds;
        encoders.resize (nj,0.0);
        accs.resize     (nj,50.0);
        spds.resize     (nj,10.0);

        if (r_pos) r_pos->setRefAccelerations (accs.data());
        if (l_pos) l_pos->setRefAccelerations (accs.data());
        if (r_pos) r_pos->setRefSpeeds        (spds.data());
        if (l_pos) l_pos->setRefSpeeds        (spds.data());

        //execute the position command
        //tuck(tuck_cmd);

        //check for auto closure
        if (rf.check("quit")) running = false;
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        if (command.get(0).asString()=="help")
        {
            reply.addString("Available commands are:");
            reply.addString("open");
            reply.addString("close");
        }
        else if (command.get(0).asString()=="open")
        {
            tuck(OPEN);
            yarp::os::Time::delay(WAIT_TIME);
            reply.addString("arms opened");
        }
        else if (command.get(0).asString()=="close")
        {
            tuck(CLOSE);
            yarp::os::Time::delay(WAIT_TIME);
            reply.addString("arms closed");
        }
        else if (command.get(0).asString()=="approach")
        {
            int dur = 14;
            approach(dur);
            yarp::os::Time::delay(WAIT_TIME);
            reply.addString("approaching complete");
        }
        else if (command.get(0).asString()=="retreat")
        {   
            int dur = 14;
            retreat(dur);
            yarp::os::Time::delay(WAIT_TIME);
            reply.addString("retreating complete");
        }
        else
        {
            reply.addString("Unknown command.");
        }
        return true;
    }

    virtual bool close()
    {
        if (left_arm_device) left_arm_device->close();
        if (right_arm_device) right_arm_device->close();
        rpcPort.close();
        port_ikart_ctrl.close();
        return true;
    }

    virtual double getPeriod()    { return 0.2;}
    virtual bool   updateModule() 
    {
        if (approach_count>0)
        {
            Bottle &b=port_ikart_ctrl.prepare();
            b.clear();
            b.addInt(3);
            b.addDouble(0.0);
            b.addDouble(0.02);
            b.addDouble(0.0);
            port_ikart_ctrl.write();
            approach_count--;
        }
        if (retreat_count>0)
        {
            Bottle &b=port_ikart_ctrl.prepare();
            b.clear();
            b.addInt(3);
            b.addDouble(0.0);
            b.addDouble(-0.02);
            b.addDouble(0.0);
            port_ikart_ctrl.write();
            retreat_count--;
        }
        return running; 
    }
};

int main(int argc, char *argv[]) 
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    iKartUtilsModule mod;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    return mod.runModule(rf);

    return 0;
}
