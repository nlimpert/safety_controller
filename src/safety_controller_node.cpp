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
#include <pluginlib/class_list_macros.h>

#include <safety_controller/safety_controller_node.h>

namespace safety_controller {
    SafetyController::SafetyController() : costmap_ros_("costmap", tf_), planning_thread_(NULL){
        ros::NodeHandle private_nh("~");
        std::cout << "INIT" << std::endl;
        private_nh.param("controller_frequency", controller_frequency_, 10.0);
        private_nh.param("num_th_samples", num_th_samples_, 20);
        private_nh.param("num_x_samples", num_x_samples_, 10);
        private_nh.param("theta_range", theta_range_, 0.7);
        private_nh.param("obs_theta_range", obs_theta_range_, 0.7);
        private_nh.param("min_radius", min_radius_, 0.27);
        private_nh.param("max_radius", max_radius_, 1.5);
        private_nh.param("min_speed", min_speed_, 0.1);
        private_nh.param("max_speed", max_speed_, 0.8);
        private_nh.param("msg_timeout", msg_timeout_sec_, 0.2);
        planner_.initialize("planner", &tf_, &costmap_ros_);

        ros::NodeHandle n;
        pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        sub_ = n.subscribe("plan_cmd_vel", 10, &SafetyController::velCB, this);
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.angular.z = 0.0;

#ifdef VISUALIZE
        marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_markers_safety_controller", 1);
#endif

        active_ = false;
        msg_timeout_ = ros::Duration(msg_timeout_sec_);

        planning_thread_ = new boost::thread(boost::bind(&SafetyController::controlLoop, this));
    }

    SafetyController::~SafetyController(){
        planning_thread_->join();
        delete planning_thread_;
    }

    void SafetyController::velCB(const geometry_msgs::TwistConstPtr& vel){
        boost::mutex::scoped_lock lock(mutex_);
        cmd_vel_ = *vel;
        last_msg_stamp_ = ros::Time::now();
        active_ = true;
    }

    void SafetyController::controlLoop(){
        ros::Rate r(controller_frequency_);
        while(ros::ok()){

            if (!active_) {
                r.sleep();
                continue;
            }

            if (ros::Time::now() - last_msg_stamp_ > msg_timeout_) {
                // stop and disable because of too old messages
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.angular.z = 0.0;
                pub_.publish(cmd);
                r.sleep();
                active_ = false;
                ROS_WARN_NAMED("safety_controller","Messages too old. Aborting");
            }

            Eigen::Vector3f desired_vel = Eigen::Vector3f::Zero();

            //we'll copy over velocity data for planning
            {
                boost::mutex::scoped_lock lock(mutex_);
                desired_vel[0] = cmd_vel_.linear.x;
                desired_vel[1] = cmd_vel_.linear.y;
                desired_vel[2] = cmd_vel_.angular.z;
            }

            bool legal = validate(desired_vel[0], desired_vel[1], desired_vel[2], true);

            if (legal) {
                geometry_msgs::Twist cmd;
                cmd.linear.x = desired_vel[0];
                cmd.linear.y = desired_vel[1];
                cmd.angular.z = desired_vel[2];
                pub_.publish(cmd);
                r.sleep();
                continue;
            } else {
                geometry_msgs::Twist cmd;
                cmd.linear.x = desired_vel[0] * 0.5;
                cmd.linear.y = desired_vel[1] * 0.5;
                cmd.angular.z = desired_vel[2];
                pub_.publish(cmd);
                r.sleep();
                continue;
            }
        }
    }

    bool SafetyController::validate(double x, double y, double th, bool update_map) {
        bool legal_traj = true;

        double dth = (obs_theta_range_) / double(num_th_samples_);

        // shift start angle to the left
        double start_th = -0.2;

#ifdef VISUALIZE
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markers;

        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!

        geometry_msgs::Point cur_point;
#endif

        for (int i = 0; i < num_th_samples_ && legal_traj; i++) {
            float cur_idth = angles::normalize_angle( i * dth - obs_theta_range_ / 2.0 + start_th);
            float cur_x = x * std::cos(cur_idth) + y * std::sin(cur_idth);
            float cur_y = y * std::cos(cur_idth) - x * std::sin(cur_idth);
            float cur_th = th + cur_idth;
            legal_traj = planner_.checkTrajectory(cur_x, cur_y, cur_th, update_map);

#ifdef VISUALIZE
            marker.id = i;
            if (i == 0) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            cur_point.x = 0.0;
            cur_point.y = 0.0;
            marker.points.push_back(cur_point);
            cur_point.x = cur_x;
            cur_point.y = cur_y;
            marker.points.push_back(cur_point);
            markers.markers.push_back(marker);
            marker.points.clear();
#endif
        }

#ifdef VISUALIZE
        marker_pub.publish(markers);
#endif

        return legal_traj;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "safety_controller");
    safety_controller::SafetyController at;
    ros::spin();
    return 0;
}
