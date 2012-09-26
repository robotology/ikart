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

#ifndef A_STAR_H
#define A_STAR_H

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
#include <list>


#include "map.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class node_type
{
	public:
	bool empty;
	int x;
	int y;
	double g_score;
	double f_score;
	cell came_from;

	node_type()
	{
		empty=true;
		x=0; 
		y=0;
		g_score=0;
		f_score=0;
		came_from.x=-1;
		came_from.y=-1;
	}

	friend bool operator<  (const node_type &a, const node_type &b);
};


bool operator < (const node_type &a, const node_type &b)
{
	return (a.f_score>b.f_score);
}

class node_map_type
{
	private:
	node_map_type()
	{
	}
	
	public:
	int w;
	int h;
	node_type** nodes;

	node_map_type(IplImage *img)
	{
		w = img->width;
		h = img->height;
		nodes = new node_type* [w];
		for (int i = 0; i < w; ++i)  nodes[i] = new node_type[h];

		cv::Mat imgMat = img; 

		for (int y=0; y<h; y++)
			for (int x=0; x<w; x++)
				{
					//beware: y and x are swapped in imgMat respect to IplImage
					if (imgMat.at<cv::Vec3b>(y,x)[0] == 254 &&
						imgMat.at<cv::Vec3b>(y,x)[1] == 254 &&
						imgMat.at<cv::Vec3b>(y,x)[2] == 254)
						nodes [x][y].empty = true;
					else
						nodes [x][y].empty = false;
					nodes [x][y].x = x;
					nodes [x][y].y = y;
				}
	}

	~node_map_type()
	{
		for (int i = 0; i < w; ++i) delete[] nodes[i];
		delete[] nodes;
	}
};

double heuristic_cost_estimate (node_type start, node_type goal)
{
	return 0;
}

class ordered_set_type
{
	vector<node_type> set;

	public:
	void insert (const node_type& t)
	{
		set.push_back(t);
		push_heap (set.begin(),set.end()); 
	}

	node_type get_smallest()
	{
		node_type t = set.front();
		pop_heap (set.begin(),set.end());	
		set.pop_back();
		return t;
	}
	void print()
	{
		printf("front (smallest)%f \n", set.front().f_score);
		for (unsigned int i=0; i<set.size(); i++)
			printf ("id%d x%d y%d %f\n", i, set[i].x, set[i].y, set[i].f_score);
		printf("back (biggest) %f \n", set.back().f_score);
	}
	int size()
	{
		return set.size();
	}
	bool find(node_type t)
	{
		for (unsigned int i=0; i<set.size(); i++)
		{
			if (set[i].x == t.x &&
				set[i].y == t.y)
				return true;
		}
		return false;
	}
};

class unordered_set_type
{
	vector<node_type> set;

	public:
	void insert (const node_type& t)
	{
		set.push_back(t);
	}
	
	bool find(node_type t)
	{
		for (unsigned int i=0; i<set.size(); i++)
		{
			if (set[i].x == t.x &&
				set[i].y == t.y)
				return true;
		}
		return false;
	}
};

bool find_astar_path(IplImage *img, cell start, cell goal, std::queue<cell>& path)
{
	if (img == 0 ) return false;

	std::vector<cell> inverse_path;
	node_map_type map(img);
	int sx=start.x;
	int sy=start.y;
	int gx=goal.x;
	int gy=goal.y;

	unordered_set_type closed_set;
	ordered_set_type   open_set;  

	open_set.insert(map.nodes[sx][sy]);
	
	map.nodes[sx][sy].g_score=0;
	map.nodes[sx][sy].f_score = map.nodes[sx][sy].g_score + heuristic_cost_estimate(map.nodes[sx][sy], map.nodes[gx][gy]);

	int iterations=0;
	while (open_set.size()>0)
	{
		iterations++;
		//printf ("%d\n", iterations++);
		//open_set.print();
		node_type curr=open_set.get_smallest();
		
		if (curr.x==goal.x &&
			curr.y==goal.y) 
			{
				cell c;
				c.x=goal.x;
				c.y=goal.y;
				while (!(c.x==start.x && c.y==start.y))
				{
					inverse_path.push_back(c);
					c.x=map.nodes[c.x][c.y].came_from.x;
					c.y=map.nodes[c.x][c.y].came_from.y;
				}
				printf("size: %d\n", inverse_path.size());

				//reverse the path				
				for (int i=inverse_path.size()-1; i>=0; i--)
				{
					path.push(inverse_path[i]);
				}
				return true;
			}

		closed_set.insert(curr);

		list<node_type> neighbors;
		neighbors.push_back(map.nodes[curr.x][curr.y+1]);
		neighbors.push_back(map.nodes[curr.x][curr.y-1]);
		neighbors.push_back(map.nodes[curr.x+1][curr.y]);
		neighbors.push_back(map.nodes[curr.x-1][curr.y]);
		neighbors.push_back(map.nodes[curr.x+1][curr.y+1]);
		neighbors.push_back(map.nodes[curr.x+1][curr.y-1]);
		neighbors.push_back(map.nodes[curr.x-1][curr.y+1]);
		neighbors.push_back(map.nodes[curr.x-1][curr.y+1]);

		while(neighbors.size()>0)
		{
			node_type neighbor = neighbors.front();

			if (closed_set.find(neighbor))
			{
				neighbors.pop_front();
				continue;
			}
			
			int nx = neighbor.x;
			int ny = neighbor.y;
			double tentative_g_score=0;
			if (neighbor.empty)
				{
					// add the distance between curr and neigh
					if ( (nx==curr.x+1 && ny==curr.y   ) ||
						 (nx==curr.x-1 && ny==curr.y   ) ||
						 (nx==curr.x   && ny==curr.y+1 ) ||
						 (nx==curr.x   && ny==curr.y-1 ) ) tentative_g_score = curr.g_score + 10;
					else 
						tentative_g_score = curr.g_score + 14;     
				}
			else
				tentative_g_score = curr.g_score + 1e10;
			
			bool b = open_set.find(neighbor);
			if (!b || tentative_g_score < map.nodes[nx][ny].g_score)
			{
				map.nodes[nx][ny].came_from.x = curr.x;
				map.nodes[nx][ny].came_from.y = curr.y;
				map.nodes[nx][ny].g_score = tentative_g_score;
				map.nodes[nx][ny].f_score = map.nodes[nx][ny].g_score + heuristic_cost_estimate( map.nodes[neighbor.x][neighbor.y],  map.nodes[gx][gy]);
				if (!b)
				{
					open_set.insert(map.nodes[nx][ny]);
				}
			}

			neighbors.pop_front();
		}
	};

	//no path found
	return false;
}




#endif
