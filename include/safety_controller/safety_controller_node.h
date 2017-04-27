/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef SAFETY_CONTROLLER_SAFETY_CONTROLLER_H_
#define SAFETY_CONTROLLER_SAFETY_CONTROLLER_H_
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <math.h>
#include <angles/angles.h>

#ifdef VISUALIZE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

namespace safety_controller {
    class SafetyController {
    public:
        SafetyController();
        ~SafetyController();
    private:
        void velCB(const geometry_msgs::TwistConstPtr& vel);

        std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius);
        bool validate(double x, double y, double th, bool update_map);

        void controlLoop();

        tf::TransformListener tf_;
        costmap_2d::Costmap2DROS costmap_ros_;
        double controller_frequency_;
        base_local_planner::TrajectoryPlannerROS planner_;
        boost::mutex mutex_;
        geometry_msgs::Twist cmd_vel_;
        boost::thread* planning_thread_;
        double theta_range_, obs_theta_range_;
        int num_th_samples_, num_x_samples_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        double min_radius_, max_radius_, min_speed_, max_speed_;
        bool active_;
        ros::Time last_msg_stamp_;
        ros::Duration msg_timeout_;
        double msg_timeout_sec_;

        double low_vel_thresh_;
        nav_msgs::Odometry odom_;
#ifdef VISUALIZE
        ros::Publisher marker_pub;
#endif
    };
};
#endif
