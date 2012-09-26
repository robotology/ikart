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
#include "aStar.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

map_class::map_class()
{
	processed_map = 0;
	loaded_map    = 0;
	tmp1          = 0;
	tmp2          = 0;
	origin.resize(3,0.0);
	crop_x        = 0;
	crop_y        = 0;
	crop_w        = 0;
	crop_h        = 0;
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

	crop_x=0;
	crop_y=0;
	crop_w=size_x=loaded_map->width;
	crop_h=size_y=loaded_map->height;

	FILE * pFile=0;
	pFile = fopen (yaml_file.c_str(),"r");
	char buff[255];
	char tmp[255];
	if (pFile!=NULL)
	{
		printf ("opening yaml map file %s\n", filename.c_str()); 
		//read here resolution, origin, size
		while (1)
		{
			int ret = fscanf(pFile,"%s", buff);
			if (ret==EOF) break;

			if (strcmp(buff,"resolution:")==0)
				{
					fscanf(pFile,"%s",tmp);
					resolution = atof(tmp);
					printf("map resolution: %f\n",resolution);
				}
			if (strcmp(buff,"origin:")==0) 
				{
					fscanf(pFile,"%s",tmp);
					origin[0] = atof(tmp+1);
					fscanf(pFile,"%s",tmp);
					origin[1] = atof(tmp);
					fscanf(pFile,"%s",tmp);
					origin[2] = atof(tmp);
					printf("map origin: [%s]\n",origin.toString().c_str());
				} 
		}
		printf("\n");
		fclose (pFile);
	}
	else
	{
		fprintf(stderr, "unable to load yaml map file %s\n", filename.c_str());
		return false;
	}

	enlargeObstacles(loaded_map, tmp1);
#define ENLARGE
#ifdef ENLARGE
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp2, tmp1);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp2, tmp1);
#endif
	crop(tmp1, processed_map);
	return true;
}

bool map_class::simplifyPath(IplImage *map, std::queue<cell> input_path, std::queue<cell>& output_path)
{
	if (map==0) return false;
	if (input_path.size()==0) return false;

	output_path.push(input_path.front());

	while (input_path.size()>0)
	{
		cell start_cell=input_path.front(); input_path.pop();
		
		std::queue<cell> tmp_path = input_path;
		while (tmp_path.size()>0)
		{		
			static cell old_stop_cell = tmp_path.front();
			cell stop_cell=tmp_path.front();  tmp_path.pop();
			if (!checkStraightLine(map, start_cell, stop_cell))
			{
				output_path.push(old_stop_cell);
				break;
			}
			old_stop_cell=stop_cell;
		};
	};
	return true;
};

void map_class::drawPath(IplImage *map, cell start, std::queue<cell> path)
{
	if (map==0) return;
	if (path.size()==0) return;
	cell src = start;
	while (path.size()>0)
	{
		cell dst = path.front();
		path.pop();
		cvLine(map, cvPoint(src.x, src.y), cvPoint(dst.x, dst.y), cvScalar(0, 150, 0));               
		src=dst;
	};
}

bool map_class::checkStraightLine(IplImage* map, cell src, cell dst)
{
	if (map==0) return false;

	//here using the fast Bresenham algorithm
	int dx=dst.x-src.x;
    int dy=dst.y-src.y;
	int D = 2*dy - dx;
	int y=src.y;
	
	cv::Mat imgMat = map; 
	for (int x=src.x+1; x<dst.x; x++)
	{
		if (D > 0)
		{
			y = y+1;
			cv::Vec3b p= imgMat.at<cv::Vec3b>(x,y);
			if (p[0] != 254 || p[1] != 254 || p[2] != 254) return false;
			D = D + (2*dy-2*dx);
		}
		else
		{
			cv::Vec3b p= imgMat.at<cv::Vec3b>(x,y);
			if (p[0] != 254 || p[1] != 254 || p[2] != 254) return false;
			D = D + (2*dy);
		}
	}
	return true;
}

bool map_class::findPath(IplImage *img, cell start, cell goal, std::queue<cell>& path)
{
	//return find_dijkstra_path(img, start, goal, path);
	return find_astar_path(img, start, goal, path);
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

	crop_x=left;
	crop_y=top;
	crop_w=right;
	crop_h=bottom;

	return true;
}

cell map_class::world2cell (yarp::sig::Vector v)
{
	cell c;
	c.x = int(v[0]/this->size_x);
	c.y = int(v[1]/this->size_y);
	return c;
}

yarp::sig::Vector map_class::cell2world (cell c)
{
	yarp::sig::Vector v(2);
	v[0] = c.x*this->size_x;
	v[1] = c.y*this->size_y;
	return v;
}