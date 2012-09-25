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
	img_map = 0;
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
		segImg->resize( this->img_map->width, this->img_map->height );
		cvCopyImage(this->img_map, (IplImage*)segImg->getIplImage());
		port->prepare() = *segImg;
		port->write();
		delete segImg;

		return true;
	}
	


	return false;
}

bool map_class::loadMap(string filename)
{
	size_x = 100;
	size_y = 100;

	string pgm_file = filename+".pgm";
	string yaml_file = filename+".yaml";

	img_map = cvLoadImage(pgm_file.c_str());
	if (img_map == 0)
	{
		fprintf(stderr, "unable to load pgm map file %s\n", filename.c_str());
		return false;
	}

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
	return true;
}