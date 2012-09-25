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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <math.h>
#include <cv.h>
#include <highgui.h> 

#include "map.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

map_class::map_class()
{
	processed_map = 0;
	loaded_map    = 0;
	tmp1          = 0;
	tmp2          = 0;
}

bool map_class::sendToPort (BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>* port)
{
	if (port!=0 && port->getOutputCount()>0)
	{
		/*yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=port->prepare();
		IplImage *ipl=(IplImage*) img.getIplImage();
		cvCopy(ipl,this->img_map);
		port->write();*/

		yarp::sig::ImageOf<yarp::sig::PixelRgb> *segImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
		segImg->resize( this->processed_map->width, this->processed_map->height );
		cvCopyImage(this->processed_map, (IplImage*)segImg->getIplImage());
		port->prepare() = *segImg;
		port->write();
		delete segImg;

		return true;
	}
	


	return false;
}

bool map_class::enlargeObstacles(IplImage* src, IplImage* dst)
{
	cv::Mat src_mat = src;
	cv::Mat dst_mat = dst;
//	cvErode(loaded_map,processed_map,0,6);
	
	for(int i=0; i<src_mat.rows; i++)
	{
		for(int j=0; j<src_mat.cols; j++) 
		{
			if ((src_mat.at<cv::Vec3b>(i,j)[0] ==  0 && src_mat.at<cv::Vec3b>(i,j)[1] ==0 && src_mat.at<cv::Vec3b>(i,j)[2] == 0) ||
				(src_mat.at<cv::Vec3b>(i,j)[0] ==255 && src_mat.at<cv::Vec3b>(i,j)[1] ==0 && src_mat.at<cv::Vec3b>(i,j)[2] == 0 ))
			{
				//dst_mat.at<cv::Vec3b>(i,j)[0]=       0; dst_mat.at<cv::Vec3b>(i,j)[1]=     0; dst_mat.at<cv::Vec3b>(i,j)[2]=     0;
				dst_mat.at<cv::Vec3b>(i,j)[0]= src_mat.at<cv::Vec3b>(i,j)[0];
				dst_mat.at<cv::Vec3b>(i,j)[1]= src_mat.at<cv::Vec3b>(i,j)[1];
				dst_mat.at<cv::Vec3b>(i,j)[2]= src_mat.at<cv::Vec3b>(i,j)[2];
				cv::Vec3b* b;
				b = &dst_mat.at<cv::Vec3b>(i-1,j);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i+1,j);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i,j-1);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i,j+1);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i-1,j-1);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i-1,j+1);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i+1,j+1);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
				b = &dst_mat.at<cv::Vec3b>(i+1,j-1);
				if ((*b)[0] == 254 && (*b)[1] == 254 && (*b)[2] == 254)
					{(*b)[0] = 255;   (*b)[1]=   0;     (*b)[2]=   0;}
			}
		}
	}

	return true;
}

bool map_class::loadMap(string filename)
{
	size_x = 100;
	size_y = 100;

	string pgm_file = filename+".pgm";
	string yaml_file = filename+".yaml";

	loaded_map = cvLoadImage(pgm_file.c_str());
	if (loaded_map == 0)
	{
		fprintf(stderr, "unable to load pgm map file %s\n", filename.c_str());
		return false;
	}
	processed_map = cvCloneImage(loaded_map);
	tmp1 = cvCloneImage(loaded_map);
	tmp2 = cvCloneImage(loaded_map);

	FILE * pFile=0;
	pFile = fopen (yaml_file.c_str(),"r");
	if (pFile!=NULL)
	{
		printf ("opening yaml map file %s\n", filename.c_str()); 
		//read here resolution, origin, size
		fclose (pFile);
	}
	else
	{
		fprintf(stderr, "unable to load yaml map file %s\n", filename.c_str());
		return false;
	}

	enlargeObstacles(loaded_map, tmp1);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp2, tmp1);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp2, tmp1);
	crop(tmp1, processed_map);
	return true;
}

bool map_class::crop(IplImage *img, IplImage *dest)
{
    int top = -1;
    int left = -1;
    int right = -1;
    int bottom = -1;

    cv::Mat imgMat = img;    
 
    for (int j=0;j<imgMat.rows;j++){
        for (int i=0;i<imgMat.cols;i++){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    top = j; 
                    goto topFound;
            }
        }
    }

    topFound:
    for (int j=imgMat.rows-1; j>0; j--){
        for (int i=imgMat.cols-1; i>0 ;i--){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    bottom = j; 
                    goto bottomFound;
            }
        }
    }

    bottomFound:
    for (int i=0;i<imgMat.cols;i++){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    left = i; 
                    goto leftFound;
            }        
       }
    }
    
    leftFound:
    for (int i=imgMat.cols-1;i>0;i--){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    right = i; 
                    goto rightFound;
            }        
       }
    }
    
    rightFound:

    cvSetImageROI (img, cvRect(left, top, right-left, bottom-top) );   
	if (dest != 0)
		cvReleaseImage (&dest);
    dest = cvCreateImage(cvSize(right-left,  bottom-top), IPL_DEPTH_8U, 3 );
    cvCopy(img, dest); 

	return true;
}