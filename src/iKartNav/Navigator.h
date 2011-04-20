#ifndef __IKARTNAV_NAVIGATOR_H__
#define __IKARTNAV_NAVIGATOR_H__

#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "Vec2D.h"
#include "CtrlThread.h"

#define DIM 200

class Navigator : public yarp::os::Thread
{
public:
    Navigator(yarp::os::ResourceFinder *rf);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

    virtual void onStop()
    {
        mKartCtrl->stop();

        mLaserPortI.interrupt();
        mTargetPortI.interrupt();
        //mCommandPortO.interrupt();

        mLaserPortI.close();
        mTargetPortI.close();
        //mCommandPortO.close();
    }

    bool checkResetAlive()
    {
        bool alive=mImAlive;
        mImAlive=false;
        return alive;
    }

    void emergencyStop()
    {
        mHaveTarget=false;
        mOmega=mOmegaRef=0.0;
        mVel=mVelRef=Vec2D::zero;

        mKartCtrl->setCtrlRef(0.0,0.0,0.0);

        /*
        yarp::os::Bottle& cmd=mCommandPortO.prepare();
        cmd.clear();
        cmd.addInt(1);
        cmd.addDouble(0.0);
        cmd.addDouble(0.0);
        cmd.addDouble(0.0);
        cmd.addDouble(65000.0); // pwm %
        mCommandPortO.write();
        */

        fprintf(stderr,"NO LASER DATA -> EMERGENCY STOP\n");
        fflush(stderr);
    }

    void shutdown()
    {
        mActive=false;
    }

protected:
    int abs(int x){ return x>0?x:-x; }

    void setOmega(double omega)
    {
        if (omega>mMaxOmega)
        {
            mOmegaRef=mMaxOmega;
        }
        else if (omega<-mMaxOmega)
        {
            mOmegaRef=-mMaxOmega;
        }
        else
        {
            mOmegaRef=omega;
        }
    }

    void setVel(const Vec2D& vel)
    {
        if (vel.mod()>mMaxSpeed)
        {
            mVelRef=vel.norm(mMaxSpeed);
        }
        else
        {
            mVelRef=vel;
        }
    }

    void setTarget(double heading,double distance)
    {
        mTarget=distance*Vec2D(-heading);
        mHaveTarget=true;
    }

    void compileZ(Vec2D* points,int N);
    void runGNF(yarp::sig::Vector& rangeData);
    void GNF(Vec2D& odoPos,double odoRot,yarp::sig::Vector& rangeData,double& direction,double& curvature);

    // config
    double mRadius;
    Vec2D mRFpos;
    double mMaxSpeed;
    double mMaxOmega;
    double mLinAcc;
    double mRotAcc;    
    int mNumPoints;
    int mNumPointsByTwo;
    double mRangeMax;
    double mAngularRes;
    double mAngleMax;
    int mPointsBuffNum;
    int mPointsBuffSize;

    Vec2D mVel;
    double mOmega;

    Vec2D mVelRef;
    double mOmegaRef;

    // scan data
    bool *mIsValid;
    Vec2D *mPoints;
    Vec2D *mRays;
    Vec2D *mPointsBuffOld;
    Vec2D *mPointsBuffNew;

    CtrlThread* mKartCtrl;

    bool mHaveTarget;
    Vec2D mTarget;

    double **mD;
    bool **mReach;
    bool **mQueued;
    double **mZeta;

    int *mQueueX;
    int *mQueueY;

    ///////////////////////
    double mSIGMA;
    double mK;
    double **mMask;
    ///////////////////////

    bool mActive;
    bool mImAlive;
    //double mTimeOld;
    //double mPeriod;

    double T[4],Tx[4],Txx[4],S[4],Sy[4],Syy[4];
    double Mb[4][4];

    yarp::os::ResourceFinder *mRF;
    yarp::os::BufferedPort<yarp::sig::Vector> mLaserPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mTargetPortI;
    //yarp::os::BufferedPort<yarp::os::Bottle> mCommandPortO;
};

#endif
