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
 * Author: Nicolas Limpert
 *********************************************************************/
#include <pluginlib/class_list_macros.h>

#include <safety_controller/safety_controller_node.h>

namespace safety_controller {
SafetyController::SafetyController() : control_thread_(NULL) {
  ros::NodeHandle private_nh("~");
  private_nh.param("controller_frequency", controller_frequency_, 10.0);
  private_nh.param("max_vel", max_vel_, 1.0);
  private_nh.param("medium_vel", med_vel_, 0.6);
  private_nh.param("low_vel", low_vel_, 0.3);
  private_nh.param("min_vel", min_vel_, 0.1);

  ros::NodeHandle n;
  laser_sub_ = n.subscribe("scan", 10, &SafetyController::laserCB, this);

  last_lin_vel = 0.0;
  control_thread_ =
      new boost::thread(boost::bind(&SafetyController::controlLoop, this));

  dynamic_reconfigure_callback =
      boost::bind(&SafetyController::reconfigCB, this, _1, _2);
  dynamic_reconfigure_server =
      new dynamic_reconfigure::Server<safety_controller::paramsConfig>(
          private_nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);
}

SafetyController::~SafetyController() {
  control_thread_->join();
  delete control_thread_;
}

void SafetyController::reconfigCB(safety_controller::paramsConfig& config,
                                  uint32_t level) {
  max_vel_ = config.max_vel_trans;
  med_vel_ = config.med_vel_trans;
  low_vel_ = config.low_vel_trans;
  max_vel_theta_ = config.max_vel_theta;

  med_dist_ = config.med_vel_dist;
  low_dist_ = config.low_vel_dist;
}

void SafetyController::laserCB(
    const sensor_msgs::LaserScanConstPtr& laser_msg) {
  boost::mutex::scoped_lock lock(laser_mutex_);
  laser_msg_ = *laser_msg;
}

void SafetyController::controlLoop() {
  ros::Rate r(controller_frequency_);
  while (ros::ok()) {
    float max_allowed_vel = max_vel_;
    float range = 0.0;
    for (int i = 0; i < laser_msg_.ranges.size(); i++) {
      range = laser_msg_.ranges[i];
      if (range <= med_dist_) {
        max_allowed_vel = med_vel_;
      }
      if (range <= low_dist_) {
        max_allowed_vel = low_vel_;
        // cancel iteration because we already found one beam to be the closest
        break;
      }
    }

    set_local_planner_max_lin_vel(max_allowed_vel);
    r.sleep();
  }
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

  if (!ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",
                          dynreconf_srv_req, dynreconf_srv_resp)) {
    ROS_WARN_THROTTLE(1.0, "Failed to send dynreconf");
    dynreconf_conf.doubles.clear();
  } else {
    ROS_INFO("Sent dynreconf to TebLocalPlannerROS, new vel: %f", new_lin_vel);
    dynreconf_conf.doubles.clear();
    last_lin_vel = new_lin_vel;
  }
}
};  // namespace safety_controller

int main(int argc, char** argv) {
  ros::init(argc, argv, "safety_controller");
  safety_controller::SafetyController at;
  ros::spin();
  return 0;
}
