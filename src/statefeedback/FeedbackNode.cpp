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
 
 
#include "FeedbackNode.h"

using namespace std;

FeedbackNode::FeedbackNode()
{
    // true -> use Gazebo data, false -> use OptiTrack data
    if (nh_.getParam("/drone_statefeedback/sim", sim)) {
    } else {
      ROS_INFO("Parameter /drone_statefeedback/sim is not found!");
      exit(1);
    }

    if (sim) {
        // Simulation enabled -> use Gazebo data
        cout << "Simulation is enabled" << endl;
        gazebo_sub_channel  = nh_.resolveName("gazebo/model_states");
        gazebo_cb           = nh_.subscribe(gazebo_sub_channel, 10, &FeedbackNode::gazeboCb, this);
        gazebo_freq         = 1000;
    } else {
        // Physical flight enabled -> use OptiTrack data
        cout << "Physical flight is enabled" << endl;
        optitrack_sub_channel   = nh_.resolveName("ardrone2/pose");
        optitrack_cb            = nh_.subscribe(optitrack_sub_channel, 10, &FeedbackNode::optitrackCb, this);
        optitrack_freq          = 120;
    }

    // Publisher with desired frequency for publishing the drone state
    pub_channel = nh_.resolveName("ardrone2_dem/state");
    pub         = nh_.advertise<tum_ardrone::filter_state>(pub_channel, 1);
    pub_freq    = 30;
}

FeedbackNode::~FeedbackNode()
{
}

void FeedbackNode::gazeboCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    gazebo_msgs::ModelStates gazebo_model_state = *msg;
    ardrone_pose = gazebo_model_state.pose[1];
    ardrone_twist = gazebo_model_state.twist[1];
}

void FeedbackNode::optitrackCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped optitrack_model_state = *msg;
    ardrone_pose = optitrack_model_state.pose;

    // convert quaternion into Euler angles
    tf::Quaternion quat;
    tf::quaternionMsgToTF(ardrone_pose.orientation, quat);
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ardrone_twist.linear = tf::Matrix3x3(quat)*;
}

void FeedbackNode::Loop()
{
    uint16_t default_freq = 1000;
    ros::Rate loop_rate(default_freq);
    if (sim) {
        loop_rate = ros::Rate(gazebo_freq);
        cnt_threshold = gazebo_freq/pub_freq;
    } else {
        loop_rate = ros::Rate(optitrack_freq);
        cnt_threshold = optitrack_freq/pub_freq;
    }
    
    cnt = 0;
    while (nh_.ok())
    {
        ros::spinOnce();
        
        if (cnt >= cnt_threshold)
        {
            // fill metadata
            tum_ardrone::filter_state s;
//            s.x = -ardrone_pose.position.y;
//            s.y = ardrone_pose.position.x;
//            s.z = ardrone_pose.position.z;
//            s.dx = ardrone_twist.linear.x;
//            s.dy = ardrone_twist.linear.y;
//            s.dz = ardrone_twist.linear.z;
//            s.roll = -ardrone_twist.angular.y;
//            s.pitch = ardrone_twist.angular.x;
//            s.yaw = -ardrone_twist.angular.z;
//            s.header.stamp = ros::Time().now();
//            s.ptamState = s.PTAM_BEST;
//            s.scaleAccuracy = 1;
//            s.droneState = lastNavdataReceived.state;
//            s.batteryPercent = lastNavdataReceived.batteryPercent;
            s.x = ardrone_pose.position.x;
            s.y = ardrone_pose.position.y;
            s.z = ardrone_pose.position.z;
//            s.dx = ardrone_twist.linear.x;    //TODO: add subscriber to navdata (linear velocities) + convert velocities measured on drone into inertial coordinate frame (mocap coordinates)
//            s.dy = ardrone_twist.linear.y;
//            s.dz = ardrone_twist.linear.z;
//            s.roll = ardrone_twist.angular.x; //TODO: add subscriber to imu data from which to derive the angular velocities + convert into intertial coordinate frame (mocap coordinates)
//            s.pitch = ardrone_twist.angular.y;
//            s.yaw = ardrone_twist.angular.z;
            s.dx = 0;
            s.dy = 0;
            s.dz = 0;
            s.roll = 0;
            s.pitch = 0;
            s.yaw = 0;
            s.header.stamp = ros::Time().now();
            s.ptamState = s.PTAM_BEST;
            s.scaleAccuracy = 1;
            
            // publish
            pub.publish(s);
            
            cnt = 0;
        }

        cnt++;
        loop_rate.sleep();
    }
}
