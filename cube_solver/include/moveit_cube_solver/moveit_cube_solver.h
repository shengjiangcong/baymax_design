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
#include <xarm_msgs/GripperConfig.h>
#include <cube_color_detector/DetectObjectSrv.h>

class CubeSolver
{
  private:
        ros::NodeHandle nh_;
        // move group运动规划组
	moveit::planning_interface::MoveGroupInterface L_xarm;
	moveit::planning_interface::MoveGroupInterface R_xarm;
	moveit::planning_interface::MoveGroupInterface xarms;

        ros::Publisher planning_scene_diff_publisher;
        moveit_msgs::PlanningScene planning_scene;

        // 存储解魔方动作的序列栈
        std::deque<std::string> cube_deque;
        const double PI = 3.141592653;

        // 夹爪的四种状态定义
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
        
        // 输入到kociemba解魔方算法服务的颜色序列，54位string
        std::string input_cube_color;
  public:
        // kociemba服务输出结果对应int数字
        std::unordered_map<std::string, int> solve_map;

	CubeSolver(ros::NodeHandle n_);

        // 添加场景模型
        void add_scene();

        // 删除场景模型
        void remove_scene();
   
        // 添加魔方模型
        void add_cube();

        // 删除魔方模型
        void remove_cube();

        // 召唤kociemba解魔方服务
        bool call_kociemba();

        // 召唤识别魔方服务
        bool call_object_detect();

        // 手臂移动到安全位置
        void move_to_safe_state();
        void L_move_to_safe_state();
        void R_move_to_safe_state();
        void R_move_to_ready_state();

        // 双臂逆运动学运动
        bool xarms_move_to(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2);

        // 左臂逆运动学运动
        bool L_xarm_move_to(geometry_msgs::Pose pos);

        // 左臂正运东学运动
        bool L_xarm_move_to(const std::vector<double> joint_group_positions);

        // 左臂单关节运动
        bool L_xarm_move_to(int index, double kind);

        // 右臂逆运动学运动
        bool R_xarm_move_to(geometry_msgs::Pose pos);

        // 右臂单关节运动
        bool R_xarm_move_to(int index, double kind);

        // 右臂正运动学运动
        bool R_xarm_move_to(const std::vector<double> joint_group_positions);

        // 无魔方单关节运动
        bool L_xarm_move_to_nocube(int index, double kind);
        bool R_xarm_move_to_nocube(int index, double kind);

        // 返回解魔方序列
        std::deque<std::string> get_cube_deque() {return cube_deque;}

        // 魔方目前在哪个手臂上，1表示右手固定魔方，2表示左手固定魔方
        int pick_num;

        // 拍照识别魔方
        bool take_photos();

        // 开始抓取魔方
        bool start_pick();

        // 夹爪控制
        void gripper_control(Gripper_mode mode);

        bool turn_U0();//顺时针90
        bool turn_U1();//逆时针90
        bool turn_U2();//180
        bool turn_UD0();
        bool turn_UD1();
        bool turn_UD2();

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

        // 切换固定魔方的手臂
        bool switch_fix_arm();

        
	
};

#endif
