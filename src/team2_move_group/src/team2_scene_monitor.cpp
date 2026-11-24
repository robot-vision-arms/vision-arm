/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

// MoveIt 2.12.4 MoveItCpp::loadPlanningSceneMonitor initialization adapted for
// Team 2's known-geometry simulation. Upstream: moveit_ros/planning/moveit_cpp/
// src/moveit_cpp.cpp (BSD-3-Clause, Copyright 2019 PickNik Inc.).
// This executable-local definition leaves the installed MoveIt library unchanged.
#include <moveit/moveit_cpp/moveit_cpp.hpp>

namespace moveit_cpp
{
bool MoveItCpp::loadPlanningSceneMonitor(const PlanningSceneMonitorOptions& options)
{
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, options.robot_description, options.name);
  if (!planning_scene_monitor_->getPlanningScene())
  {
    RCLCPP_ERROR(logger_, "Planning scene not configured");
    return false;
  }
  planning_scene_monitor_->startStateMonitor(options.joint_state_topic, options.attached_collision_object_topic);
  planning_scene_monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, options.monitored_planning_scene_topic);
  planning_scene_monitor_->startSceneMonitor(options.publish_planning_scene_topic);

  // 작업 셀은 SDF에서 읽은 충돌 물체로 검사한다. 깊이 센서가 없는 모드에서는
  // Octomap만 초기화하지 않으며, 관절·부착 물체·장면 변경 감시는 그대로 유지한다.
  // 실제 깊이 센서를 연결할 때는 이 매개변수를 false로 설정하고 센서를 구성한다.
  bool known_geometry_only = true;
  node_->get_parameter_or("team2_known_geometry_only", known_geometry_only, true);
  planning_scene_monitor_->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      !known_geometry_only);
  RCLCPP_INFO(logger_, "Team 2 world monitor: %s",
              known_geometry_only ? "known collision geometry (Octomap disabled)" : "Octomap enabled");
  return options.wait_for_initial_state_timeout <= 0.0 ||
         planning_scene_monitor_->getStateMonitor()->waitForCurrentState(
             node_->now(), options.wait_for_initial_state_timeout);
}
}  // namespace moveit_cpp
