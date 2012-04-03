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
    CtrlThread(std::string local) 
        : RateThread(10),mCtrlSem(1),mLocal(local)
    {
        mOmega=0.0;
        mSpeed=0.0;
        mHead=0.0;
    }

    virtual bool threadInit()
    {
        mCommandPortO.open((mLocal+"/control:o").c_str());

        return true;
    }
    
    virtual void run()
    {
        mCtrlSem.wait();
        
        sendCommand(mHead,mSpeed,mOmega);

        mCtrlSem.post();
    }
    
    void onStop()
    {
        sendCommand(0.0,0.0,0.0);
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

    std::string mLocal;
    yarp::os::Semaphore mCtrlSem;
    yarp::os::BufferedPort<yarp::os::Bottle> mCommandPortO;

    void sendCommand(double head,double speed,double omega)
    {
        if (mCommandPortO.getOutputCount()>0)
        {
            yarp::os::Bottle& cmd=mCommandPortO.prepare();
            cmd.clear();
            cmd.addInt(2);
            cmd.addDouble(head);
            cmd.addDouble(speed);
            cmd.addDouble(omega);
            mCommandPortO.write();
        }
    }
};

#endif


