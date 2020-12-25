/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef PROBOT_CUBE_SOLVER
#define PROBOT_CUBE_SOLVER

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include "cube_solver/solve_list.h"
#include <sstream>
#include <deque>

class CubeSolver
{
  private:
        ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface L_xarm;
	moveit::planning_interface::MoveGroupInterface R_xarm;
	moveit::planning_interface::MoveGroupInterface xarms;

        ros::Publisher planning_scene_diff_publisher;
        moveit_msgs::PlanningScene planning_scene;
        std::deque<std::string> cube_deque;//存储解的序列
  public:
	CubeSolver(ros::NodeHandle n_);
        void add_scene();
        void remove_scene();
        bool call_kociemba();
        void move_to_safe_state();
        bool xarms_move_to(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2);
        bool L_xarm_move_to(geometry_msgs::Pose pos);
        bool R_xarm_move_to(geometry_msgs::Pose pos);
        std::deque<std::string> get_cube_deque() {return cube_deque;}
	
};

#endif
