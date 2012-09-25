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

#ifndef MAP_H
#define MAP_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <cv.h>
#include <highgui.h> 

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class map_class
{
	int size_x;
	int size_y;
	double                                  resolution;
	yarp::sig::Vector                       origin;
	IplImage*                               img_map;
	yarp::sig::ImageOf<yarp::sig::PixelRgb> data_map;
	
	public:
	map_class();

	bool loadMap(string filename);
	bool sendToPort (BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>* port); 
};

#endif
