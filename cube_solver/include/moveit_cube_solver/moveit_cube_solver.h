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
#include <locale>
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
#include <unordered_map>
#include <xarm_msgs/GripperMove.h>


//
//right:横向;-0.707,0,0,0.707
//      向下：1,0,0,0
//      向前：0.707，0，0.707，0
//      向上：0，0，0，1
//

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
        const double PI = 3.141592653;
        enum Gripper_mode
        {
          L_closed,
          L_open,
          R_closed,
          R_open
        };
    moveit_msgs::CollisionObject cube;
    shape_msgs::SolidPrimitive cube_primitive;
    geometry_msgs::Pose cube_pose;
  public:
        std::unordered_map<std::string, int> solve_map;
	CubeSolver(ros::NodeHandle n_);
        void add_scene();
        void remove_scene();
        bool call_kociemba();
        void move_to_safe_state();
        void L_move_to_safe_state();
        void R_move_to_safe_state();
        void R_move_to_ready_state();
        bool xarms_move_to(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2);
        bool L_xarm_move_to(geometry_msgs::Pose pos);
        bool L_xarm_move_to(int index, double kind);
        bool R_xarm_move_to(geometry_msgs::Pose pos);
        bool R_xarm_move_to(int index, double kind);
        std::deque<std::string> get_cube_deque() {return cube_deque;}
        int pick_num;//1表示右手固定魔方，2表示左手
        bool start_pick();
        void gripper_control(Gripper_mode mode);
        void gripper_control_test();
        bool turn_U0();//顺时针90
        bool turn_U1();//逆时针90
        bool turn_U2();//180

        bool turn_L0();//顺时针90
        bool turn_L1();//逆时针90
        bool turn_L2();//180

        bool turn_D0();//顺时针90
        bool turn_D1();//逆时针90
        bool turn_D2();//180

        bool turn_F0();//顺时针90
        bool turn_F1();//逆时针90
        bool turn_F2();//180

        bool turn_R0();//顺时针90
        bool turn_R1();//逆时针90
        bool turn_R2();//180

        bool turn_B0();//顺时针90
        bool turn_B1();//逆时针90
        bool turn_B2();//180

        bool switch_fix_arm();//切换固定魔方的手臂

        
	
};

#endif
