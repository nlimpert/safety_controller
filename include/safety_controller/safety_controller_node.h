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
#ifndef SAFETY_CONTROLLER_SAFETY_CONTROLLER_H_
#define SAFETY_CONTROLLER_SAFETY_CONTROLLER_H_
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <ros/ros.h>
#include <safety_controller/paramsConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

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
  void laserCB(const sensor_msgs::LaserScanConstPtr& laser_msg);

  dynamic_reconfigure::Server<safety_controller::paramsConfig>*
      dynamic_reconfigure_server;
  dynamic_reconfigure::Server<safety_controller::paramsConfig>::CallbackType
      dynamic_reconfigure_callback;
  void reconfigCB(safety_controller::paramsConfig& config, uint32_t level);

  void controlLoop();
  void set_local_planner_max_lin_vel(double new_lin_vel);

  double controller_frequency_;
  boost::mutex laser_mutex_;
  sensor_msgs::LaserScan laser_msg_;

  boost::thread* control_thread_;
  ros::Subscriber laser_sub_;
  double min_radius_, max_radius_, min_speed_, max_speed_;

  double min_vel_, max_vel_, med_vel_, low_vel_, max_vel_theta_;
  double med_dist_, low_dist_;
};
};  // namespace safety_controller
#endif
