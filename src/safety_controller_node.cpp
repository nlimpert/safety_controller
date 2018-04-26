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
        private_nh.param("controller_frequency", controller_frequency_, 10.0);
        private_nh.param("num_th_samples", num_th_samples_, 20);
        private_nh.param("num_x_samples", num_x_samples_, 10);
        private_nh.param("theta_range", theta_range_, 0.7);
        private_nh.param("obs_theta_range", obs_theta_range_, 0.7);
        private_nh.param("min_radius", min_radius_, 0.27);
        private_nh.param("max_radius", max_radius_, 1.5);
        private_nh.param("min_speed", min_speed_, 0.1);
        private_nh.param("high_vel", high_vel_, 1.0);
        private_nh.param("medium_vel", medium_vel_, 0.6);
        private_nh.param("low_vel", low_vel_, 0.3);

        private_nh.param("high_vel_cost_thresh", high_vel_cost_thresh_, 20);
        private_nh.param("medium_vel_cost_thresh", medium_vel_cost_thresh_, 30);
        private_nh.param("low_vel_cost_thresh", low_vel_cost_thresh_, 70);

        private_nh.param("msg_timeout", msg_timeout_sec_, 0.2);
        planner_.initialize("planner", &tf_, &costmap_ros_);

        ros::NodeHandle n;
        pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        sub_ = n.subscribe("plan_cmd_vel", 10, &SafetyController::velCB, this);
        laser_sub_ = n.subscribe("scan", 10, &SafetyController::laserCB, this);
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.angular.z = 0.0;
        last_lin_vel = 0.0;

#ifdef VISUALIZE
        marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_markers_safety_controller", 1);
#endif

        active_ = false;
        msg_timeout_ = ros::Duration(msg_timeout_sec_);
        last_msg_stamp_ = ros::Time::now();

        planning_thread_ = new boost::thread(boost::bind(&SafetyController::controlLoop, this));
    }

    SafetyController::~SafetyController(){
        planning_thread_->join();
        delete planning_thread_;
    }

    void SafetyController::velCB(const geometry_msgs::TwistConstPtr& vel){
        boost::mutex::scoped_lock lock(vel_mutex_);
        cmd_vel_ = *vel;
        last_msg_stamp_ = ros::Time::now();
        active_ = true;
        costmap_ros_.start();
    }

    void SafetyController::laserCB(const sensor_msgs::LaserScanConstPtr& laser_msg){
        boost::mutex::scoped_lock lock(laser_mutex_);
        laser_msg_ = *laser_msg;
    }

    void SafetyController::controlLoop(){
        ros::Rate r(controller_frequency_);
        while(ros::ok()){

            if (!active_) {
                costmap_ros_.stop();
                r.sleep();
                continue;
            }

            // update pose
            if (!costmap_ros_.getRobotPose(robot_pose))
            {
              ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
            }

            costmap_ros_.getCostmap()->worldToMap(
                  robot_pose.getOrigin().x(),
                  robot_pose.getOrigin().y(),
                  robot_pose_x,
                  robot_pose_y);

            float max_allowed_vel = medium_vel_;
            robot_pose_cell_cost = costmap_ros_.getCostmap()->getCost(robot_pose_x, robot_pose_y);

            if (robot_pose_cell_cost <= medium_vel_cost_thresh_) {
              max_allowed_vel = high_vel_;
            } else if (robot_pose_cell_cost > medium_vel_cost_thresh_ &&
                       robot_pose_cell_cost < low_vel_cost_thresh_) {
              max_allowed_vel = medium_vel_;
            }


            for (int i = 0; i < laser_msg_.ranges.size(); i++) {
              float r = laser_msg_.ranges[i];
              if (r <= 1.0) {
                max_allowed_vel = low_vel_;
                break;
              }
            }

            set_local_planner_max_lin_vel(max_allowed_vel);

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
                boost::mutex::scoped_lock lock(vel_mutex_);
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

    void SafetyController::set_local_planner_max_lin_vel(double new_lin_vel) {
      if (new_lin_vel == last_lin_vel) {
        return;
      }
      dynamic_reconfigure::ReconfigureRequest dynreconf_srv_req;
      dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;
      dynamic_reconfigure::DoubleParameter dynreconf_max_vel_x_param;
      dynamic_reconfigure::DoubleParameter dynreconf_max_vel_y_param;
      dynamic_reconfigure::BoolParameter dynreconf_bool_param;
      dynamic_reconfigure::Config dynreconf_conf;

      dynreconf_max_vel_x_param.name = "max_vel_x";
      dynreconf_max_vel_x_param.value = new_lin_vel;
      dynreconf_max_vel_y_param.name = "max_vel_y";
      dynreconf_max_vel_y_param.value = new_lin_vel;
      dynreconf_conf.doubles.push_back(dynreconf_max_vel_x_param);
      dynreconf_conf.doubles.push_back(dynreconf_max_vel_y_param);
      dynreconf_srv_req.config = dynreconf_conf;

      if (! ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", dynreconf_srv_req, dynreconf_srv_resp)) {
          ROS_ERROR("Failed to send dynreconf");
          dynreconf_conf.doubles.clear();
      } else {
          ROS_INFO("Sent dynreconf to TebLocalPlannerROS, new vel: %f", new_lin_vel);
          dynreconf_conf.doubles.clear();
          last_lin_vel = new_lin_vel;

      }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "safety_controller");
    safety_controller::SafetyController at;
    ros::spin();
    return 0;
}
