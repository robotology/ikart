/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include "filters.h"

double ikart_filters::lp_filter_1Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off freqency of 1Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /1.689454484e+01;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.8816185924 * yv[0][i]);
    return yv[1][i];
}

double ikart_filters::lp_filter_0_5Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off freqency of 0.5Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /3.282051595e+01;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.9390625058 * yv[0][i]);
    return yv[1][i];
}

double ikart_filters::ratelim_filter_0(double input, int i)
{
    //This is a rate limiter filter. 
    static double prev[3];
    if      (input>prev[i]+10) prev[i]=prev[i]+10;
    else if (input<prev[i]-10) prev[i]=prev[i]-10;
    else     prev[i]=input;
    return prev[i];
}
