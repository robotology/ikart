#ifndef __IKARTNAV_NAVTHREAD_H__
#define __IKARTNAV_NAVTHREAD_H__

#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Vector.h>

#include "Vec2D.h"

class CtrlThread : public yarp::os::RateThread
{
public:
    CtrlThread(std::string local,std::string remote) 
        : RateThread(10),mCtrlSem(1)
    {
        mLocal=local;
        mRemote=remote;

        mOmega=0.0;
        mSpeed=0.0;
        mHead=0.0;
    }

    virtual bool threadInit()
    {
        mCommandPortO.open((mLocal+"/control:o").c_str());
        
        /*
        if (!yarp::os::Network::connect(mCommandPortO.getName(),(mRemote+"/control:i").c_str()))
        {
            fprintf(stderr,"ERROR: can't connect to iKartCtrl command port\n");
        }
        */

        return true;
    }
    
    virtual void run()
    {
        mCtrlSem.wait();
    
        if (mCommandPortO.getOutputCount()>0)
        {
            // SEND COMMANDS
            yarp::os::Bottle& cmd=mCommandPortO.prepare();
            cmd.clear();
            cmd.addInt(1);
            cmd.addDouble(mHead);
            cmd.addDouble(mSpeed);
            cmd.addDouble(mOmega);
            cmd.addDouble(65000.0); // pwm %
            mCommandPortO.write();
        }

        mCtrlSem.post();
    }
    
    virtual void threadRelease()
    {
        mCommandPortO.interrupt();
        mCommandPortO.close();
    }

    void setCtrlRef(double head,double speed,double omega)
    {
        mCtrlSem.wait();

        mOmega=omega;
        mSpeed=speed;
        mHead=head;
        
        mCtrlSem.post();
    }

protected:
    double mOmega;
    double mSpeed;
    double mHead;

    yarp::os::BufferedPort<yarp::os::Bottle> mCommandPortO;

    yarp::os::Semaphore mCtrlSem;

    std::string mLocal;
    std::string mRemote;
};

#endif

