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
#include "TargetFilter.h"

#define DIM      128
#define DIM_BY_2  64

#define THR       0.666   // 0.666
#define SAFETY    1.5     // 1.25

//#define THR       0.7  // 0.666
//#define SAFETY    1.5  // 1.25

class Navigator : public yarp::os::RateThread
{
public:
    Navigator(yarp::os::ResourceFinder *rf);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

    void emergencyStop()
    {
        mOmega=mOmegaRef=0.0;
        mVel=mVelRef=Vec2D::zero;
        mKartCtrl->setCtrlRef(0.0,0.0,0.0);
    }

    void updateOdometry(Vec2D P,double H)
    {
        mOdoP=P;
        mOdoH=H;

        int xc=XWorld2Grid(mOdoP.x);
        int yc=YWorld2Grid(mOdoP.y);

        if (xc> DIM_BY_2) shiftMapSouth();
        if (xc<-DIM_BY_2) shiftMapNorth();

        if (yc> DIM_BY_2) shiftMapEast();
        if (yc<-DIM_BY_2) shiftMapWest();
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

    inline int XWorld2Grid(double& x){ return int(10.0*x)-mOx; }
    inline int YWorld2Grid(double& y){ return int(10.0*y)-mOy; }

    inline int XWorld2GridRound(double& x){ return int(10.0*x+0.5)-mOx; }
    inline int YWorld2GridRound(double& y){ return int(10.0*y+0.5)-mOy; }

    inline double XGrid2World(int x){ return 0.1*double(x+mOx); }
    inline double YGrid2World(int y){ return 0.1*double(y+mOy); }
    inline Vec2D Grid2World(int x,int y) { return Vec2D(0.1*double(x+mOx),0.1*double(y+mOy)); }

    void setVisionTarget(double heading)
    {
        int angle=RAYS_BY_2+int(0.5+4.0*mod180(heading));
        
        mTargetFilter.addPoint(mOdoP,Vec2D(heading+mOdoH),mRanges[angle]);

        mHaveTarget=false;

        if (mTargetFilter.numSamples()<2)
        {
            printf("Target not ready\n");
            return;
        }
        
        mTargetOriginal=mTargetFilter.getTarget();

        mTarget=mTargetOriginal;
        
        printf("Target X=%.3f   Y=%.3f\n",mTarget.x,mTarget.y);

        int xT=XWorld2GridRound(mTarget.x);
        int yT=YWorld2GridRound(mTarget.y);

        if (!mReach[xT][yT])
        {
            mHaveTarget=true;
            return;
        }
    
        replaceTarget();
    }    

    void setUserTarget(double heading,double distance)
    {
        mTargetOriginal=mOdoP+distance*Vec2D(mOdoH+heading);

        mTarget=mTargetOriginal;

        mHaveTarget=false;

        int xT=XWorld2GridRound(mTarget.x);
        int yT=YWorld2GridRound(mTarget.y);

        if (!mReach[xT][yT])
        {
            mHaveTarget=true;
            return;
        }
    
        replaceTarget();
    }
    
    void replaceTarget() 
    {    
        double Dmax=(mTarget-mOdoP).mod()+0.1;

        int xT=XWorld2GridRound(mTarget.x);
        int yT=YWorld2GridRound(mTarget.y);

        int xM=xT,yM=yT;

        while (mReach[xT][yT])
        {
            xT=xM;
            yT=yM;

            int min=mReach[xT][yT];

            if (xT>-DIM && mReach[xT-1][yT  ]<min && (Grid2World(xT-1,yT  )-mOdoP).mod()<=Dmax) min=mReach[xM=xT-1][yM=yT  ];
            if (xT< DIM && mReach[xT+1][yT  ]<min && (Grid2World(xT+1,yT  )-mOdoP).mod()<=Dmax) min=mReach[xM=xT+1][yM=yT  ];
            if (yT>-DIM && mReach[xT  ][yT-1]<min && (Grid2World(xT  ,yT-1)-mOdoP).mod()<=Dmax) min=mReach[xM=xT  ][yM=yT-1]; 
            if (yT< DIM && mReach[xT  ][yT+1]<min && (Grid2World(xT  ,yT+1)-mOdoP).mod()<=Dmax) min=mReach[xM=xT  ][yM=yT+1];

            if (xT>-DIM && yT>-DIM && mReach[xT-1][yT-1]<min && (Grid2World(xT-1,yT-1)-mOdoP).mod()<=Dmax) min=mReach[xM=xT-1][yM=yT-1];
            if (xT< DIM && yT>-DIM && mReach[xT+1][yT-1]<min && (Grid2World(xT+1,yT-1)-mOdoP).mod()<=Dmax) min=mReach[xM=xT+1][yM=yT-1];
            if (xT>-DIM && yT< DIM && mReach[xT-1][yT+1]<min && (Grid2World(xT-1,yT+1)-mOdoP).mod()<=Dmax) min=mReach[xM=xT-1][yM=yT+1];
            if (xT< DIM && yT< DIM && mReach[xT+1][yT+1]<min && (Grid2World(xT+1,yT+1)-mOdoP).mod()<=Dmax) min=mReach[xM=xT+1][yM=yT+1];

            if (xT==xM && yT==yM) break;
        }

        if (xT==xM && yT==yM)
        {
            Vec2D U=(mOdoP-mTarget).norm();

            for (double d=0.0; d<Dmax; d+=0.1)
            {
                Vec2D T=mTarget+d*U;
                xT=XWorld2GridRound(T.x);
                yT=YWorld2GridRound(T.y);
                
                if (xT<-DIM) xT=-DIM; else if (xT>DIM-1) xT=DIM-1;
                if (yT<-DIM) yT=-DIM; else if (yT>DIM-1) yT=DIM-1;

                if (!mReach[xT][yT])
                {
                    mHaveTarget=true;
                    mTarget=Grid2World(xT,yT);
                    printf("Target replaced X=.3%f   Y=%.3f\n",mTarget.x,mTarget.y);
                    if (!mHaveTargetH)
                    {
                        mHaveTargetH=true;
                        mTargetH=(mTargetOriginal-mTarget).arg();
                    }
                    return;
                }
            }
        }
        else
        {
            mHaveTarget=true;
            mTarget=Grid2World(xT,yT);
            printf("Target replaced X=.3%f   Y=%.3f\n",mTarget.x,mTarget.y);
            if (!mHaveTargetH)
            {
                mHaveTargetH=true;
                mTargetH=(mTargetOriginal-mTarget).arg();
            }
        }
    }
    
    void addEvent(double heading,double distance,double radius);

    void updateMap(yarp::sig::Vector& rangeData);
    void updateZeta();
    void updateGNF();
    bool followGNF(Vec2D &W,double& curvature,double &zeta,Vec2D &gradient);

    void shiftMapSouth();
    void shiftMapNorth();
    void shiftMapWest();
    void shiftMapEast();

    // config
    double mRadius;
    Vec2D mRFpos;
    double mMinSpeed;
    double mMaxSpeed;
    double mMaxOmega;
    double mLinAcc;
    double mRotAcc;    
    double mRangeMax;
    double mAngularRes;
    double mAngleMax;

    Vec2D  mOdoP;
    double mOdoH;

    int mOx,mOy;

    Vec2D mVel;
    double mOmega;

    Vec2D mVelRef;
    double mOmegaRef;

    Vec2D *mRays;
    double *mRanges;
    int RAYS;
    int RAYS_BY_2;

    CtrlThread* mKartCtrl;

    bool mHaveTarget;
    Vec2D mTarget;
    Vec2D mTargetOriginal;
    bool mHaveTargetH;
    double mTargetH;
    bool mMustStop;

    double **mD;
    unsigned short **mReach;
    bool **mQueued;
    double **mZeta;
    unsigned char **mPrio;

    double **mSamples;

    int *mQueueX;
    int *mQueueY;

    ///////////////////////
    double mK;
    double **mMask;
    ///////////////////////

    TargetFilter mTargetFilter;

    //bool mImAlive;

    double T[4],Tx[4],Txx[4],S[4],Sy[4],Syy[4];
    double Mb[4][4];

    yarp::os::ResourceFinder *mRF;
    
    yarp::os::Port mResetOdometryPortO;
    yarp::os::Port mStatusPortO;

    yarp::os::BufferedPort<yarp::sig::Vector> mLaserPortI;

    yarp::os::BufferedPort<yarp::os::Bottle> mOdometryPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mUserPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mVisionPortI;
};

#endif

