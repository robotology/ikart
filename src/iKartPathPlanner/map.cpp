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
	origin.resize(3,0.0);
	crop_x        = 0;
	crop_y        = 0;
	crop_w        = 0;
	crop_h        = 0;
}

bool map_class::sendToPort (BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>* port, IplImage* image_to_send)
{
	if (port!=0 && port->getOutputCount()>0)
	{
		yarp::sig::ImageOf<yarp::sig::PixelRgb> *segImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
		segImg->resize(image_to_send->width, image_to_send->height );
		cvCopyImage(image_to_send, (IplImage*)segImg->getIplImage());
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
	cv::Vec3b white (254,254,254);
	cv::Vec3b red   (255,0,0);
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
				int il = i-1>0?i-1:0;
				int ir = i+1<src_mat.rows-1?i+1:src_mat.rows-1;
				int ju = j-1>0?j-1:0;
				int jd = j+1<src_mat.cols-1?j+1:src_mat.cols-1;
				printf ("-- %d %d %d %d\n", il, ir, ju, jd);
				b = &dst_mat.at<cv::Vec3b>(il,j);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(ir,j);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(i,ju);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(i,jd);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(il,ju);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(il,jd);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(ir,jd);
				if ((*b) == white) (*b) = red;
				b = &dst_mat.at<cv::Vec3b>(ir,ju);
				if ((*b) == white) (*b) = red;
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

	
	IplImage *cropped_map = 0;
	
	crop(loaded_map, cropped_map);
	cvReleaseImage (&loaded_map);
	loaded_map = cropped_map;

	IplImage*  tmp1 = cvCloneImage(loaded_map);
	IplImage*  tmp2 = cvCloneImage(loaded_map);
	processed_map   = cvCloneImage(loaded_map);

	enlargeObstacles(loaded_map, tmp1);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp2, tmp1);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp1, tmp2);
	enlargeObstacles(tmp2, processed_map);
	
	cvReleaseImage (&tmp1);
	cvReleaseImage (&tmp2);
	return true;
}

bool map_class::simplifyPath(IplImage *map, std::queue<cell> input_path, std::queue<cell>& output_path)
{
	if (map==0) return false;
	if (input_path.size()==0) return false;

	output_path.push(input_path.front());

	for (unsigned int i=0; i<input_path.size(); i++)
	{
		cell start_cell=input_path._Get_container().at(i);
		cell old_stop_cell;
		cell stop_cell;
		unsigned int j;
		for (j=i+1; j<input_path.size(); j++)
		{
			old_stop_cell = input_path._Get_container().at(j-1);
			stop_cell     = input_path._Get_container().at(j);
			//printf ("%d %d -> %d %d\n", start_cell.x, start_cell.y, stop_cell.x, stop_cell.y);
			if (!checkStraightLine(map, start_cell, stop_cell))
			{
				break;
			}		
		};
		if (j==input_path.size())
		{
			output_path.push(stop_cell);
			return true;
		}
		else
		{
			output_path.push(old_stop_cell);
			i=j-1;
		}
	};
	return true;
};

void map_class::drawPath(IplImage *map, cell start, std::queue<cell> path, const CvScalar& color)
{
	if (map==0) return;
	if (path.size()==0) return;
	cell src = start;
	while (path.size()>0)
	{
		cell dst = path.front();
		path.pop();
		cvLine(map, cvPoint(src.x, src.y), cvPoint(dst.x, dst.y), color);               
		src=dst;
	};
}

void map_class::drawCurrentPosition(IplImage *map, cell current, const CvScalar& color)
{
	if (map==0) return;
	cvCircle(map, cvPoint(current.x, current.y), 6, color);               
}

bool map_class::checkStraightLine(IplImage* map, cell src, cell dst)
{
	if (map==0) return false;

	//here using the fast Bresenham algorithm
	int dx = abs(dst.x-src.x);
    int dy = abs(dst.y-src.y); 
	int err = dx-dy;
	
	int sx;
	int sy;
    if (src.x < dst.x) sx = 1; else sx = -1;
    if (src.y < dst.y) sy = 1; else sy = -1;
	
	cv::Mat imgMat = map; 
	while(1)
	{
		cv::Vec3b p= imgMat.at<cv::Vec3b>(src.y,src.x);		
	    if (p[0] != 254 || p[1] != 254 || p[2] != 254) return false;
		if (src.x==dst.x && src.y==src.y) break;
		int e2 = err*2;
		if (e2 > -dy)
		{
			err = err-dy;
			src.x += sx;
		}
		if (e2 < dx)
		{
			err = err+dx;
			src.y += sy;
		}
	}
	return true;
}

bool map_class::findPath(IplImage *img, cell start, cell goal, std::queue<cell>& path)
{
	//return find_dijkstra_path(img, start, goal, path);
	return find_astar_path(img, start, goal, path);
}

bool map_class::crop(IplImage *img, IplImage* &dest)
{
    int top = -1;
    int left = -1;
    int right = -1;
    int bottom = -1;

    cv::Mat imgMat = img;    
 
    for (int j=0;j<imgMat.rows;j++){
        for (int i=0;i<imgMat.cols;i++){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    top = j; 
                    goto topFound;
            }
        }
    }

    topFound:
    for (int j=imgMat.rows-1; j>0; j--){
        for (int i=imgMat.cols-1; i>0 ;i--){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    bottom = j+1; 
                    goto bottomFound;
            }
        }
    }

    bottomFound:
    for (int i=0;i<imgMat.cols;i++){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    left = i; 
                    goto leftFound;
            }        
       }
    }
    
    leftFound:
    for (int i=imgMat.cols-1;i>0;i--){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
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
    cvCopyImage(img, dest); 

	crop_x=left;
	crop_y=top;
	crop_w=right;
	crop_h=bottom;

	return true;
}

cell map_class::world2cell (yarp::sig::Vector v)
{
	cell c;
	c.x = int(v[0]/this->resolution);
	c.y = int(v[1]/this->resolution);
	return c;
}

yarp::sig::Vector map_class::cell2world (cell c)
{
	yarp::sig::Vector v(2);
	v[0] = c.x*this->resolution;
	v[1] = c.y*this->resolution;
	return v;
}