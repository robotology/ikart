#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "bridgeModule.h"
#include "bridgeThread.h"

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);
    rf.setDefaultContext("ikart");
    rf.setDefaultConfigFile("bridgeConf.ini");

    if (rf.check("help"))
    {
        printf("\n");
        printf("Available options:");
        printf("--laser_resample <n>\n");
        printf("--no_odom_tf\n");
        printf("\n");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    BridgeModule mod;

    return mod.runModule(rf);
}

