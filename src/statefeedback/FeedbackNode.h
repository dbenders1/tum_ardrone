#pragma once
 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __FEEDBACKNODE_H
#define __FEEDBACKNODE_H
 

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"


struct FeedbackNode
{
private:
	  ros::NodeHandle nh_;

	  // input
    ros::Subscriber gazebo_cb;
    uint16_t gazebo_freq;
    ros::Subscriber optitrack_cb;
    uint16_t optitrack_freq;

	  // output
	  geometry_msgs::Pose ardrone_pose;
	  geometry_msgs::Twist ardrone_twist;
    ros::Publisher pub;
    double roll, pitch, yaw;

	  // parameters
	  bool sim;
	  std::string gazebo_sub_channel;
	  std::string optitrack_sub_channel;
	  std::string pub_channel;
    int pub_freq;

    // misc
    uint16_t cnt, cnt_threshold;

public:
	  FeedbackNode();
	  ~FeedbackNode();

	  // ROS message callbacks
	  void gazeboCb(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void optitrackCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

	  // main pose feedback loop
	  void Loop();
};


#endif /* __FEEDBACKNODE_H */
