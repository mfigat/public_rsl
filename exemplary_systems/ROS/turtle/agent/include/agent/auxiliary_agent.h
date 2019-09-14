/*
	Copyright (c) 2019, Robot Control and Pattern Recognition Group, Warsaw University of Technology
	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
				* Redistributions of source code must retain the above copyright
				  notice, this list of conditions and the following disclaimer.
				* Redistributions in binary form must reproduce the above copyright
				  notice, this list of conditions and the following disclaimer in the
				  documentation and/or other materials provided with the distribution.
				* Neither the name of the Warsaw University of Technology nor the
				  names of its contributors may be used to endorse or promote products
				  derived from this software without specific prior written permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	Author: Maksym Figat

*/

#ifndef AUXILIARY_AGENT_H
#define AUXILIARY_AGENT_H

 /* Some includes */
#include "ros/ros.h"
#include <ros/duration.h>
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h> 

#define DEFAULT_ROS_FREQUENCY 1
#define CHANNEL_SIZE 1
#define GO_NORTH 1
#define GO_EAST 2
#define GO_SOUTH 3
#define GO_WEST 4 
#define EPSILON_ROTATE 0.00001
#define EPSILON_MOVE 0.05
#define ROTATE_ANGLE 1.5
#define MOVE_ALONG_SPEED 2.5
#define GO_NORTH_DIRECTION 1.57079632679
#define GO_EAST_DIRECTION 0
#define GO_SOUTH_DIRECTION -1.57079632679
#define GO_WEST_DIRECTION 3.14159265359

#endif /* END OF HEADER */

