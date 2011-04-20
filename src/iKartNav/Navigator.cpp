//#include "stdafx.h"

#include <yarp/os/Time.h>

#include "Navigator.h"

//Navigator::Navigator(iKartCtrl* kartCtrl) 
Navigator::Navigator(yarp::os::ResourceFinder *rf)
    :  //mKartCtrl(kartCtrl),
        mRF(rf),
        mVel(0.0,0.0),
        mOmega(0.0),
        mVelRef(0.0,0.0),
        mOmegaRef(0.0)
{
    mActive=true;

    mIsValid=NULL;
    mPoints=NULL;
    mPointsBuffOld=NULL;
    mPointsBuffNew=NULL;

    int SIZE=2*DIM+1;

    mD=new double*[SIZE];
    mD[0]=new double[SIZE*SIZE];

    mZeta=new double*[SIZE];
    mZeta[0]=new double[SIZE*SIZE];

    mReach=new bool*[SIZE];
    mReach[0]=new bool[SIZE*SIZE];
    
    mQueued=new bool*[SIZE];
    mQueued[0]=new bool[SIZE*SIZE];

    for (int i=1; i<SIZE; ++i)
    {
        mD[i]=mD[i-1]+SIZE;
        mZeta[i]=mZeta[i-1]+SIZE;
        mReach[i]=mReach[i-1]+SIZE;
        mQueued[i]=mQueued[i-1]+SIZE;
    }
    
    mQueueX=new int[SIZE*SIZE];
    mQueueY=new int[SIZE*SIZE];

    ///////////////////////////////////

    mMask=new double*[21];
    mMask[0]=new double[21*21];
    for (int i=1; i<21; ++i)
    {
        mMask[i]=mMask[i-1]+21;
    }

    int Mx,My;
    mSIGMA=0.5;
    mK=-0.5/(mSIGMA*mSIGMA);
    for (int x=-10; x<=10; ++x)
	{
        Mx=x+10;
        double dX=0.1*double(x);
	    for (int y=-10; y<=10; ++y)
        {
            My=y+10;
	        double dY=0.1*double(y);
	        mMask[Mx][My]=exp(mK*(dX*dX+dY*dY));
	    }
	}

    T[3]  =S[3]  =1.0;
    Tx[2] =Sy[2] =1.0;
    Txx[1]=Syy[1]=2.0;
    Tx[3] =Sy[3] =0.0;
    Txx[3]=Syy[3]=0.0;
    Txx[2]=Syy[2]=0.0;
    
    T[2]=1.0/3.0;   S[2]=1.0/3.0;
    T[1]=T[2]*T[2]; S[1]=S[2]*S[2];
    T[0]=T[2]*T[1]; S[0]=S[2]*S[1];
    
    Tx[0] =3.0*T[1]; Sy[0] =3.0*S[1];
    Tx[1] =2.0*T[2]; Sy[1] =2.0*S[2];
    Txx[0]=6.0*T[2]; Syy[0]=6.0*S[2];

    Mb[0][0]=-1.0; Mb[0][1]= 3.0; Mb[0][2]=-3.0; Mb[0][3]=1.0;
    Mb[1][0]= 3.0; Mb[1][1]=-6.0; Mb[1][2]= 3.0; Mb[1][3]=0.0;
    Mb[2][0]=-3.0; Mb[2][1]= 3.0; Mb[2][2]= 0.0; Mb[2][3]=0.0;
    Mb[3][0]= 1.0; Mb[3][1]= 0.0; Mb[3][2]= 0.0; Mb[3][3]=0.0;
}

bool Navigator::threadInit()
{
    mRadius=mRF->check("radius",yarp::os::Value(0.3575)).asDouble();
    mRFpos.x=mRF->check("laser_pos",yarp::os::Value(0.245)).asDouble();
    mMaxSpeed=mRF->check("max_speed",yarp::os::Value(0.125)).asDouble();
    mMaxOmega=mRF->check("max_ang_speed",yarp::os::Value(30.0)).asDouble();
    mLinAcc=mRF->check("linear_acc",yarp::os::Value(0.25)).asDouble();
    mRotAcc=mRF->check("rot_acc",yarp::os::Value(60.0)).asDouble();
    mNumPoints=mRF->check("num_range_samples",yarp::os::Value(1081)).asInt();
    mRangeMax=mRF->check("range_max_dist",yarp::os::Value(9.5)).asDouble();
    mAngularRes=mRF->check("range_ang_res",yarp::os::Value(0.25)).asDouble();

    //mRadius=0.3575;
    //mRFpos.x=0.245;
    mRFpos.y=0.0;
    //mMaxSpeed=0.125;  // m/s
    //mMaxOmega=30.0;  // deg/s
    //mLinAcc=0.25;    // m/s2
    //mRotAcc=60.0;    // deg/s2
    //mNumPoints=1081;
    //mRangeMax=9.5;
    //mAngularRes=0.25;

    mNumPointsByTwo=mNumPoints/2;
    mAngleMax=double(mNumPointsByTwo)*mAngularRes;
    mPointsBuffSize=3*((int)(360.0/mAngularRes)-mNumPoints);

    ///////////////////////////////////

    mPointsBuffNum=0;
    mPoints=new Vec2D[mNumPoints];
    mRays=new Vec2D[mNumPoints];
    mPointsBuffOld=new Vec2D[mPointsBuffSize];
    mPointsBuffNew=new Vec2D[mPointsBuffSize];
    mIsValid=new bool[mNumPoints];
    
    for (int i=0; i<mNumPoints; ++i)
    {
        mIsValid[i]=false;
        mRays[i]=Vec2D(double(i-mNumPointsByTwo)*mAngularRes);
    }

    std::string remote=mRF->check("remote",yarp::os::Value("/ikart")).asString().c_str();
    std::string local=mRF->check("local",yarp::os::Value("/ikartnav")).asString().c_str();
        
    mLaserPortI.open((local+"/laser:i").c_str());
    if (!yarp::os::Network::connect((remote+"/laser:o").c_str(),mLaserPortI.getName()))
    {
        fprintf(stderr,"ERROR: can't connect to iKartCtrl laser port\n");
        return false;
    }

    mTargetPortI.open((local+"/target:i").c_str());

    /*
    mCommandPortO.open((local+"/control:o").c_str());
    if (!yarp::os::Network::connect(mCommandPortO.getName(),(remote+"/control:i").c_str()))
    {
        fprintf(stderr,"ERROR: can't connect to iKartCtrl command port\n");
        return false;
    }
    */

    mKartCtrl=new CtrlThread(local,remote);

    mKartCtrl->start();

    return true;
}

void Navigator::threadRelease()
{
    mKartCtrl->stop();
    delete mKartCtrl;

    mLaserPortI.close();
    mTargetPortI.close();
    //mCommandPortO.close();

    if (mRays) delete [] mRays;
    if (mPoints) delete [] mPoints;
    if (mIsValid) delete [] mIsValid;
    if (mPointsBuffOld) delete [] mPointsBuffOld;
    if (mPointsBuffNew) delete [] mPointsBuffNew;

    mRays=NULL;
    mPoints=NULL;
    mIsValid=NULL;
    mPointsBuffOld=NULL;
    mPointsBuffNew=NULL;

    delete [] mD[0];
    delete [] mD;

    delete [] mZeta[0];
    delete [] mZeta;

    delete [] mReach[0];
    delete [] mReach;

    delete [] mQueued[0];
    delete [] mQueued;

    delete [] mQueueX;
    delete [] mQueueY;

    delete [] mMask[0];
    delete [] mMask;
}

void Navigator::GNF(Vec2D& odoPos,double odoRot,yarp::sig::Vector& rangeData,double& direction,double& curvature)
{
    //mLaserSem.wait();

    for (int i=0; i<mNumPoints; ++i)
    {
        if (mIsValid[i])
        {
            mPoints[i]-=odoPos;
            mPoints[i]=mPoints[i].rot(-odoRot);
        }
    }

    for (int i=0; i<mPointsBuffNum; ++i)
    {
        mPointsBuffOld[i]-=odoPos;
        mPointsBuffOld[i]=mPointsBuffOld[i].rot(-odoRot);
    }

    static const double cosAngleMax=cos(Vec2D::DEG2RAD*(mAngleMax));

    int pointsBuffNum=0;
    for (int i=mNumPoints-1; i>mNumPointsByTwo; --i)
    {
        if (mIsValid[i])
        {
            if ((mPoints[i]-mRFpos).norm().x<=cosAngleMax)
            {
                mPointsBuffNew[pointsBuffNum++]=mPoints[i];
            }
            else
            {
                break;
            }
        }
    }

    for (int i=0; i<mNumPointsByTwo; ++i)
    {
        if (mIsValid[i])
        {
            if ((mPoints[i]-mRFpos).norm().x<=cosAngleMax)
            {
                mPointsBuffNew[pointsBuffNum++]=mPoints[i];
            }
            else
            {
                break;
            }
        }
    }

    for (int i=0; i<mPointsBuffNum && pointsBuffNum<mPointsBuffSize; ++i)
    {
        if ((mPointsBuffOld[i]-mRFpos).norm().x<=cosAngleMax)
        {
            mPointsBuffNew[pointsBuffNum++]=mPointsBuffOld[i];
        }
    }

    mPointsBuffNum=pointsBuffNum;
    Vec2D *swap=mPointsBuffOld;
    mPointsBuffOld=mPointsBuffNew;
    mPointsBuffNew=swap;

    double range;
    for (int i=0; i<mNumPoints; ++i)
    {
        range=0.001*rangeData[i];

        if (range<mRangeMax)
        {
            mPoints[i]=mRFpos+range*mRays[i];
            mIsValid[i]=true;
        }
        else
        {
            mIsValid[i]=false;
        }
    }

    // compile map here

    int Mx,My;
    for (int x=-DIM; x<=DIM; ++x)
    {
        Mx=x+DIM;
        for (int y=-DIM; y<=DIM; ++y)
        {
            My=y+DIM;

            mD[Mx][My]=1E+10;
            mZeta[Mx][My]=0.0;
            mReach[Mx][My]=false;
            mQueued[Mx][My]=false;
        }
    }

    compileZ(mPoints,mNumPoints);
    compileZ(mPointsBuffOld,mPointsBuffNum);

    int head=0,tail=0;

    int xT=(int)(10.0*mTarget.x);
    int yT=(int)(10.0*mTarget.y);
    
    if (xT<-DIM) xT=-DIM; else if (xT>DIM-1) xT=DIM-1;
    if (yT<-DIM) yT=-DIM; else if (yT>DIM-1) yT=DIM-1;

    int MxT=xT+DIM;
    int MyT=yT+DIM;

    double dXl,dXr,dYd,dYu;
    dXl=0.1*(10.0*mTarget.x-double(xT));
    dYd=0.1*(10.0*mTarget.y-double(yT));
    dXr=0.1-dXl;
    dYu=0.1-dYd;

    mD[MxT]    [MyT]=sqrt(dXl*dXl+dYd*dYd);
    mD[MxT+1]  [MyT]=sqrt(dXr*dXr+dYd*dYd);
    mD[MxT]  [MyT+1]=sqrt(dXl*dXl+dYu*dYu);
    mD[MxT+1][MyT+1]=sqrt(dXr*dXr+dYu*dYu);

    static const double THR=0.666;

    mQueueX[tail  ]=xT;
    mQueueY[tail++]=yT;

    mQueued[MxT][MyT]=true;
    mReach[MxT][MyT]=(mZeta[MxT][MyT]<THR);

    mQueueX[tail  ]=xT+1;
    mQueueY[tail++]=yT;
    
    mQueued[MxT+1][MyT]=true;
    mReach[MxT+1][MyT]=(mZeta[MxT+1][MyT]<THR);

    mQueueX[tail  ]=xT;
    mQueueY[tail++]=yT+1;

    mQueued[MxT][MyT+1]=true;
    mReach[MxT][MyT+1]=(mZeta[MxT][MyT+1]<THR);

    mQueueX[tail  ]=xT+1;
    mQueueY[tail++]=yT+1;

    mQueued[MxT+1][MyT+1]=true;
    mReach[MxT+1][MyT+1]=(mZeta[MxT+1][MyT+1]<THR);

    static const int MODULE=(2*DIM+1)*(2*DIM+1);
    int xc,yc;
    int Mxc,Myc;
    while(head!=tail)
	{
        head%=MODULE;

	    xc=mQueueX[head  ];
	    yc=mQueueY[head++];
        
        Mxc=xc+DIM;
        Myc=yc+DIM;

	    mQueued[Mxc][Myc]=false;
	  
        double dD=mD[Mxc][Myc];
	    double dNewDL=1.25*0.1*mZeta[Mxc][Myc]+0.1;
	    double dNewDT=1.414*dNewDL+dD;
	    dNewDL+=dD;

        int xa=xc-1; if (xa<-DIM) xa=-DIM;
	    int xb=xc+1; if (xb> DIM) xb= DIM;
	    int ya=yc-1; if (ya<-DIM) ya=-DIM; 
	    int yb=yc+1; if (yb> DIM) yb= DIM;

        int Mx,My;

	    for (int x=xa; x<=xb; ++x)
        {
            Mx=x+DIM;
            for (int y=ya; y<=yb; ++y)
            {
                My=y+DIM;
                if (x!=xc || y!=yc)
	            {
	                double dNewD=(x==xc || y==yc)?dNewDL:dNewDT;

	                if (mReach[Mxc][Myc] && !mReach[Mx][My] || mReach[Mx][My]==mReach[Mxc][Myc] &&  mD[Mx][My]>dNewD)
		            {
		                mD[Mx][My]=dNewD;

		                if (mZeta[Mx][My]<THR) mReach[Mx][My]=mReach[Mxc][Myc];

		                if (!mQueued[Mx][My])
		                {
		                    tail%=MODULE;
		                    mQueueX[tail  ]=x;
		                    mQueueY[tail++]=y;
		                    mQueued[Mx][My]=true;
                        }
                    }
		        }
		    }
	    }
	}

    static double MbD[4][4],MbDMb[4][4];
    
    for (int t=0; t<4; ++t)
    {
        for (int s=0; s<4; ++s)
        {
            MbD[t][s]=0.0;
            for (int k=0; k<4-t; ++k)
            {
                MbD[t][s]-=Mb[t][k]*mD[DIM+k-1][DIM+s-1];
            }
        }
    }

    for (int t=0; t<4; ++t)
    {
        for (int s=0; s<4; ++s)
        {
            MbDMb[t][s]=0.0;
            for (int k=0; k<4-s; ++k)
            {
                MbDMb[t][s]+=MbD[t][k]*Mb[k][s];
            }
        }
    }

        static double A[4],B[4],By[4];

    for (int t=0; t<4; ++t)
    {
        A[t]=B[t]=By[t]=0.0;
        for (int s=0; s<4; ++s)
        {
            A[t]+=T[s]*MbDMb[s][t];
            B[t]+=MbDMb[t][s]*S[s];
            By[t]+=MbDMb[t][s]*Sy[s];
        }
    }

    Vec2D W(Tx[0]*B[0]+Tx[1]*B[1]+Tx[2]*B[2],Sy[0]*A[0]+Sy[1]*A[1]+Sy[2]*A[2]);
    double modW=W.normalize();

    double Wxx=Txx[0]*B[0]+Txx[1]*B[1];
    double Wxy=Tx[0]*By[0]+Tx[1]*By[1]+Tx[2]*By[2];
    double Wyy=Syy[0]*A[0]+Syy[1]*A[1];

    //double dGamma=Atan(gx,gy);
    //double dOmega=dAlfa*(dVx*(Gxy*gx-Gxx*gy)+dVy*(Gyy*gx-Gxy*gy))/(gx*gx+gy*gy);

    static const double ONE_BY_03=1.0/(0.1*3.0);

    direction=W.arg();
    curvature=ONE_BY_03*(W.x*(W.x*Wxy-W.y*Wxx)+W.y*(W.x*Wyy-W.y*Wxy))/modW;
    //mLaserSem.post();
}

void Navigator::compileZ(Vec2D* points,int N)
{
    int xc,yc;
    double dXl,dXr,dYd,dYu;
    double dExl,dExr,dEyd,dEyu;
    double px,py;

    for (int i=0; i<N; ++i)
    {
        xc=(int)(px=10.0*points[i].x);
        yc=(int)(py=10.0*points[i].y);

        if (abs(xc)>DIM || abs(yc)>DIM) continue;

        dXl=0.1*(px-double(xc));
        dYd=0.1*(py-double(yc));
        dXr=0.1-dXl;
        dYu=0.1-dYd;

        dExl=exp(mK*dXl*dXl);
        dExr=exp(mK*dXr*dXr);
        dEyd=exp(mK*dYd*dYd);
        dEyu=exp(mK*dYu*dYu);

        int x,y;
        for (int dx=-10; dx<=10; ++dx) if (abs(x=xc+dx)<=DIM)
        {
            x+=DIM;
            for (int dy=-10; dy<=10; ++dy) if (abs(y=yc+dy)<=DIM)
            {
                y+=DIM;
                double dZ=mMask[dx+10][dy+10]*(dx<=0?dExl:dExr)*(dy<=0?dEyd:dEyu);

                if (mZeta[x][y]<dZ)
                {
                    mZeta[x][y]=dZ;
                }
            }
        }
    }
}

void Navigator::run()
{
    yarp::sig::Vector *rangeData;

    while (mActive)
    {
        for (yarp::os::Bottle* bot; bot=mTargetPortI.read(false);)
        {
            yarp::os::ConstString cmd=bot->get(0).asString();

            if (cmd=="target")
            { 
                double heading=-bot->get(1).asDouble();
                double distance=bot->get(2).asDouble();
                mTarget=distance*Vec2D(heading);
                mHaveTarget=true;

                printf("NEW TARGET H=%lf D=%lf\n",heading,distance);
                fflush(stdout);
            }
            else if (cmd=="stop" || cmd=="s")
            {
                printf("STOP\n");
                fflush(stdout);
                mHaveTarget=false;
            }
        }

        rangeData=mLaserPortI.read();
        
        if (rangeData!=NULL && mActive && !isStopping())
        {   
            mImAlive=true;
            runGNF(*rangeData);
        }
    }
}

void Navigator::runGNF(yarp::sig::Vector& rangeData)
{
    // (very poor) ODOMETRY
    // only used for target memory
    
    double timeNow=yarp::os::Time::now();
    static double timeOld=timeNow;
    double step=timeNow-timeOld;
    timeOld=timeNow;
    static double period=0.0;
    period+=step;
    
    static double odoRot=0.0;
    static Vec2D odoPos;
    odoPos+=step*mVel.rot(0.5*step*mOmega);
    odoRot+=step*mOmega;
    while (odoRot>=180.0) odoRot-=360.0;
    while (odoRot<-180.0) odoRot+=360.0;

    //if (mPeriod<0.1) return;
    //printf("period %lf\n",step);
    //fflush(stdout);

    // CONTROL
    if (period>0.1)
    {
        period=0.0;

        if (mHaveTarget)
        {
            // target direction
            mTarget-=odoPos;
            mTarget=mTarget.rot(-odoRot);
            double distance=mTarget.mod();

            static int sTime=0;
            if (++sTime==10)
            {
                sTime=0;
                printf("%.3lf meters to target\n",distance);
                fflush(stdout);
            }

            double heading;
            double curvature;

            GNF(odoPos,odoRot,rangeData,heading,curvature);

            Vec2D U(heading);

            if (distance<0.05)
            {
                setVel(Vec2D::zero);
                setOmega(0.0);
            }
            else if (distance<0.2)
            {
                setVel(0.5*distance*mTarget.norm());
                //setOmega(0.2*mTarget.arg());
                setOmega(0.0);
            }
            else
            {
                if (fabs(heading)<=45.0)
                {
                    setVel(mMaxSpeed*U);
                    //static const double RAD2DEG=1.0/Vec2D::DEG2RAD;
                    setOmega(0.2*heading/*+RAD2DEG*mVel.mod()*curvature*/);
                }
                else
                {
                    setVel(Vec2D::zero);
                    setOmega(heading>0.0?9.0:-9.0);
                } 
            }
        }
        else
        {
            setVel(Vec2D::zero);
            setOmega(0.0);
        }

        odoRot=0.0;
        odoPos=Vec2D::zero;
    }

    // SMOOTHING
    if (mVelRef!=mVel)
    {
        Vec2D Verr=mVelRef-mVel;
        if (Verr.mod()>step*mLinAcc)
            mVel+=Verr.norm(step*mLinAcc);
        else
            mVel=mVelRef;
    }

    if (mOmegaRef!=mOmega)
    {
        double Werr=mOmegaRef-mOmega;

        if (Werr>step*mRotAcc)
            mOmega+=step*mRotAcc;
        else if (Werr<-step*mRotAcc)
            mOmega-=step*mRotAcc;
        else
            mOmega=mOmegaRef;
    }

    // SEND COMMANDS
    mKartCtrl->setCtrlRef(-mVel.arg(),127500.0*mVel.mod(),-1000.0*mOmega); 

    /*
    //if (mCommandPortO.getOutputCount()>0)
    {
        // SEND COMMANDS
        yarp::os::Bottle& cmd=mCommandPortO.prepare();
        cmd.clear();
        cmd.addInt(1);
        cmd.addDouble(-mVel.arg());
        cmd.addDouble(127500.0*mVel.mod());
        //cmd.addDouble(45000.0);
        cmd.addDouble(-1000.0*mOmega);
        cmd.addDouble(65000.0); // pwm %
        mCommandPortO.write();
    }
    */
}

/*
void Navigator::draw(CDC* pDC,Vec2D &P,double H)
{
    static const double s=1440.0/20.0;

    static CPen rPen(PS_SOLID,1,RGB(255,0,0));
    static CPen gPen(PS_SOLID,1,RGB(0,255,0));

    int x,y;

    CPen *pen=pDC->SelectObject(&gPen);

    Vec2D Q;

    for (int i=0; i<mNumPoints; ++i)
    {
        if (mIsValid[i])
        {
            Q=P+mPoints[i].rot(H);

            x=720-int(s*Q.y);
            y=450-int(s*Q.x);
            pDC->Ellipse(x-2,y-2,x+2,y+2);
        }
    }

    pDC->SelectObject(&rPen);
    for (int i=0; i<mPointsBuffNum; ++i)
    {
        Q=P+mPointsBuffOld[i].rot(H);

        x=720-int(s*Q.y);
        y=450-int(s*Q.x);
        pDC->Ellipse(x-2,y-2,x+2,y+2);
    }

    //x=720-int(s*mOdoPos.y);
    //y=450-int(s*mOdoPos.x);

    pDC->SelectObject(pen);
}
*/
