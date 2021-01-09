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

#include "moveit_cube_solver/moveit_cube_solver.h"

CubeSolver::CubeSolver(ros::NodeHandle n_) :
    L_xarm("L_xarm6"), 
    R_xarm("R_xarm6"), 
    xarms("xarm6s"),
    pick_num(-1)
{
    this->nh_ = n_;

    planning_scene_diff_publisher = this->nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      sleep_t.sleep();
    }

    std::string end_effector_link = L_xarm.getEndEffectorLink();

    std::string reference_frame = "ground";
    L_xarm.setPoseReferenceFrame(reference_frame);

    L_xarm.allowReplanning(true);

    L_xarm.setGoalPositionTolerance(0.001);
    L_xarm.setGoalOrientationTolerance(0.01);

    L_xarm.setMaxAccelerationScalingFactor(0.5);
    L_xarm.setMaxVelocityScalingFactor(0.5);

    solve_map = {{"U", 0}, {"U'", 1}, {"U2", 2}, 
                 {"L", 3}, {"L'", 4}, {"L2", 5}, 
                 {"D", 6}, {"D'", 7}, {"D2", 8},
                 {"F", 9}, {"F'", 10}, {"F2", 11},
                 {"R", 12}, {"R'", 13}, {"R2", 14},
                 {"B", 15}, {"B'", 16}, {"B2", 17}};
}

void CubeSolver::add_scene()
{
     // 声明一个障碍物体
    moveit_msgs::CollisionObject cube;
    cube.id = "cube";
    cube.header.frame_id = "ground";

    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive cube_primitive;
    cube_primitive.type = cube_primitive.BOX;
    cube_primitive.dimensions.resize(3);
    cube_primitive.dimensions[0] = 0.055;//魔方长宽高
    cube_primitive.dimensions[1] = 0.055;
    cube_primitive.dimensions[2] = 0.055;

    // 设置障碍物的位置
    geometry_msgs::Pose cube_pose;
    cube_pose.orientation.w = 1.0;
    cube_pose.position.x = 0;//魔方位置xyz
    cube_pose.position.y = 0;
    cube_pose.position.z = 0;//-0.25

    // 将障碍物的属性、位置加入到障碍物的实例中
    cube.primitives.push_back(cube_primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = cube.ADD;

     // 声明一个障碍物体
    moveit_msgs::CollisionObject bottom_wall;
    bottom_wall.id = "bottom_wall";
    bottom_wall.header.frame_id = "ground";

    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive bottom_wall_primitive;
    bottom_wall_primitive.type = bottom_wall_primitive.BOX;
    bottom_wall_primitive.dimensions.resize(3);
    bottom_wall_primitive.dimensions[0] = 10;
    bottom_wall_primitive.dimensions[1] = 10;
    bottom_wall_primitive.dimensions[2] = 0.01;

    // 设置障碍物的位置
    geometry_msgs::Pose bottom_wall_pose;
    bottom_wall_pose.orientation.w = 1.0;
    bottom_wall_pose.position.x = 0;
    bottom_wall_pose.position.y = 0;
    bottom_wall_pose.position.z = -0.5;//地面的高度

    // 将障碍物的属性、位置加入到障碍物的实例中
    bottom_wall.primitives.push_back(bottom_wall_primitive);
    bottom_wall.primitive_poses.push_back(bottom_wall_pose);
    bottom_wall.operation = bottom_wall.ADD;

        // 所有障碍物加入列表后，再把障碍物加入到当前的情景中
    planning_scene.world.collision_objects.push_back(bottom_wall);
    planning_scene.world.collision_objects.push_back(cube);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
}

bool CubeSolver::call_kociemba()
{
    ros::ServiceClient client = nh_.serviceClient<cube_solver::solve_list>("kociemba");
    cube_solver::solve_list srv;
    srv.request.flag = true;
    if (client.call(srv))
    {
      std::string list = srv.response.list;
      ROS_INFO("call kociemba service success!");
      //std::cout << list << std::endl;
      if (list.size() == 0)
         return false;
      std::istringstream tmp(list);
      while (tmp >> list)
      {
           cube_deque.push_back(list);
           //std::cout << list << std::endl;
      }
      return true;
    }
    else
    {
      ROS_ERROR("fail to call kociemba service!");
      return false;
    }

}

void CubeSolver::move_to_safe_state()
{
    xarms.setNamedTarget("safe_state");
    xarms.move();
    sleep(1);
}

void CubeSolver::L_move_to_safe_state()
{
    L_xarm.setNamedTarget("safe_state");
    L_xarm.move();
    sleep(1);
}

void CubeSolver::R_move_to_safe_state()
{
    R_xarm.setNamedTarget("safe_state");
    R_xarm.move();
    sleep(1);
}

void CubeSolver::remove_scene()
{
    // 声明障碍物体
    moveit_msgs::CollisionObject remove_bottom_wall;
    remove_bottom_wall.id = "bottom_wall";
    remove_bottom_wall.header.frame_id = "ground";
    remove_bottom_wall.operation = remove_bottom_wall.REMOVE;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_bottom_wall);
    planning_scene_diff_publisher.publish(planning_scene);
}

bool CubeSolver::xarms_move_to(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2)
{
      xarms.setStartStateToCurrentState();
      xarms.setPoseTarget(pos1,"L_link6");
      xarms.setPoseTarget(pos2,"R_link6");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::planning_interface::MoveItErrorCode success = xarms.plan(plan);
      sleep(1);
    
      if (!success)
         return false;
      success = xarms.execute(plan);
      sleep(1);
      if (!success)
         return false;
      else
         return true;
}

bool CubeSolver::L_xarm_move_to(geometry_msgs::Pose pos)
{
      L_xarm.setStartStateToCurrentState();
      L_xarm.setPoseTarget(pos);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::planning_interface::MoveItErrorCode success = L_xarm.plan(plan);
      sleep(1);
    
      if (!success)
         return false;
      success = L_xarm.execute(plan);
      sleep(1);
      if (!success)
         return false;
      else
         return true;
}

bool CubeSolver::R_xarm_move_to(geometry_msgs::Pose pos)
{
      R_xarm.setStartStateToCurrentState();
      R_xarm.setPoseTarget(pos);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::planning_interface::MoveItErrorCode success = R_xarm.plan(plan);
      sleep(1);
    
      if (!success)
         return false;
      success = R_xarm.execute(plan);
      sleep(1);
      if (!success)
         return false;
      else
         return true;
}

bool CubeSolver::L_xarm_move_to(int index, double kind)
{
    std::vector<double> joint_group_positions = L_xarm.getCurrentJointValues();
    joint_group_positions[index] += kind * PI / 2.0;
    L_xarm.setJointValueTarget(joint_group_positions);
    L_xarm.move();
}

bool CubeSolver::R_xarm_move_to(int index, double kind)
{
    std::vector<double> joint_group_positions = R_xarm.getCurrentJointValues();
    joint_group_positions[index] += kind * PI / 2.0;
    R_xarm.setJointValueTarget(joint_group_positions);
    R_xarm.move();
}

bool CubeSolver::start_pick()
{
    ROS_INFO("准备抓起魔方。");

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0.0;
    target_pose_r.position.y = 0.0;
    target_pose_r.position.z = 0.05;
    target_pose_r.orientation.x = 1.0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;

    target_pose_r.position.z = -0.05;
   
    if(R_xarm_move_to(target_pose_r) == false)
       return false;

    Gripper_mode mode = R_closed;
    gripper_control(mode);

    target_pose_r.position.x = 0.0;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0;
    target_pose_r.orientation.x = -0.5;
    target_pose_r.orientation.y = -0.5;
    target_pose_r.orientation.z = -0.5;
    target_pose_r.orientation.w = 0.5;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
    pick_num = 1;//右手用于固定魔方，左手旋转
    add_scene();

return true;

}

void CubeSolver::gripper_control(Gripper_mode mode)
{
if (mode == L_closed)
{
    ROS_INFO("左手夹爪闭合");
}
else if (mode == L_open)
{
    ROS_INFO("左手夹爪张开");
}
else if (mode == R_closed)
{
    ROS_INFO("右手夹爪闭合");
}
else if (mode == R_open)
{
    ROS_INFO("右手夹爪张开");
}
else 
{
    ROS_ERROR("夹爪模式错误");
}
//sleep(3);

}



bool CubeSolver::turn_U0()//-1.325110706084886
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
ROS_INFO("顺时针旋转上面90度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0;
    target_pose_l.position.z = 0.20;
    target_pose_l.orientation.x = 1;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,1);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_U1()
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
ROS_INFO("逆时针旋转上面90度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0;
    target_pose_l.position.z = 0.20;
    target_pose_l.orientation.x = 1;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,-1);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_U2()
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}
if (pick_num == 1)
{
ROS_INFO("旋转上面180度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0;
    target_pose_l.position.z = 0.20;
    target_pose_l.orientation.x = 1;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,2);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_L0()//-1.5653094231344482
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
    ROS_INFO("顺时针旋转左面90度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.2;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = -0.5;
    target_pose_l.orientation.y = -0.5;
    target_pose_l.orientation.z = 0.5;
    target_pose_l.orientation.w = -0.5;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,1);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_L1()
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
    ROS_INFO("逆时针旋转左面90度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.2;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = -0.5;
    target_pose_l.orientation.y = -0.5;
    target_pose_l.orientation.z = 0.5;
    target_pose_l.orientation.w = -0.5;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,-1);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_L2()
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
    ROS_INFO("旋转左面180度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.2;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = -0.5;
    target_pose_l.orientation.y = -0.5;
    target_pose_l.orientation.z = 0.5;
    target_pose_l.orientation.w = -0.5;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,2);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_D0()//-0.2840679308264519
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
    ROS_INFO("顺时针旋转下面90度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0;
    target_pose_l.position.z = -0.2;
    target_pose_l.orientation.x = 0;
    target_pose_l.orientation.y = 0;
    target_pose_l.orientation.z = -0.7071068;
    target_pose_l.orientation.w = 0.7071068;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,1);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_D1()
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
    ROS_INFO("逆时针旋转下面90度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0;
    target_pose_l.position.z = -0.2;
    target_pose_l.orientation.x = 0;
    target_pose_l.orientation.y = 0;
    target_pose_l.orientation.z = -0.7071068;
    target_pose_l.orientation.w = 0.7071068;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,-1);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_D2()
{
if (pick_num != 1)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 1)
{
    ROS_INFO("旋转下面180度。");
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0;
    target_pose_l.position.z = -0.2;
    target_pose_l.orientation.x = 0;
    target_pose_l.orientation.y = 0;
    target_pose_l.orientation.z = -0.7071068;
    target_pose_l.orientation.w = 0.7071068;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
L_xarm_move_to(5,2);
gripper_control((Gripper_mode)(L_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_F0()//-1.3174343380298432
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}
if (pick_num == 2)
{
    ROS_INFO("顺时针旋转前面90度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0;
    target_pose_r.position.z = -0.2;
    target_pose_r.orientation.x = 0;
    target_pose_r.orientation.y = 0;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 1;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5,1);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_F1()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 2)
{
    ROS_INFO("逆时针旋转前面90度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0;
    target_pose_r.position.z = -0.2;
    target_pose_r.orientation.x = 0;
    target_pose_r.orientation.y = 0;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 1;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5,-1);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_F2()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}
if (pick_num == 2)
{
    ROS_INFO("旋转前面180度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0;
    target_pose_r.position.z = -0.2;
    target_pose_r.orientation.x = 0;
    target_pose_r.orientation.y = 0;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 1;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5,2);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_R0()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 2)
{
    ROS_INFO("顺时针旋转右面90度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = -0.2;
    target_pose_r.position.z = 0;
    target_pose_r.orientation.x = -0.5;
    target_pose_r.orientation.y = -0.5;
    target_pose_r.orientation.z = -0.5;
    target_pose_r.orientation.w = 0.5;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5,1);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_R1()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 2)
{
    ROS_INFO("逆时针旋转右面90度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = -0.2;
    target_pose_r.position.z = 0;
    target_pose_r.orientation.x = -0.5;
    target_pose_r.orientation.y = -0.5;
    target_pose_r.orientation.z = -0.5;
    target_pose_r.orientation.w = 0.5;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5, -1);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_R2()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 2)
{
    ROS_INFO("旋转右面180度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = -0.2;
    target_pose_r.position.z = 0;
    target_pose_r.orientation.x = -0.5;
    target_pose_r.orientation.y = -0.5;
    target_pose_r.orientation.z = -0.5;
    target_pose_r.orientation.w = 0.5;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5, 2);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_B0()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}


if (pick_num == 2)
{
    ROS_INFO("顺时针旋转后面90度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0;
    target_pose_r.position.z = 0.2;
    target_pose_r.orientation.x = 0.7071068;
    target_pose_r.orientation.y = 0.7071068;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5,1);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_B1()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}

if (pick_num == 2)
{
    ROS_INFO("逆时针旋转后面90度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0;
    target_pose_r.position.z = 0.2;
    target_pose_r.orientation.x = 0.7071068;
    target_pose_r.orientation.y = 0.7071068;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5, -1);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::turn_B2()
{
if (pick_num != 2)
{
   if (switch_fix_arm() == false)
   {
      return false;
   }
}
if (pick_num == 2)
{
    ROS_INFO("旋转后面180度。");
gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0;
    target_pose_r.position.z = 0.2;
    target_pose_r.orientation.x = 0.7071068;
    target_pose_r.orientation.y = 0.7071068;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
R_xarm_move_to(5, 2);
gripper_control((Gripper_mode)(R_open));//张开左夹爪
return true;
}
else
return false;
}

bool CubeSolver::switch_fix_arm()
{
if (pick_num == 1)
{
gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = 0.7071068;
    target_pose_l.orientation.y = 0;
    target_pose_l.orientation.z = 0;
    target_pose_l.orientation.w = 0.7071068;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    gripper_control((Gripper_mode)(R_open));//张开右夹爪
    R_move_to_safe_state();
    L_xarm_move_to(5,1);
    pick_num = 2;
    return true;
}
else if (pick_num == 2)
{    
    R_move_to_safe_state();
    L_xarm_move_to(5, -1);
gripper_control((Gripper_mode)(R_open));//张开右夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0.0;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0;
    target_pose_r.orientation.x = -0.5;
    target_pose_r.orientation.y = -0.5;
    target_pose_r.orientation.z = -0.5;
    target_pose_r.orientation.w = 0.5;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
    gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
    gripper_control((Gripper_mode)(L_open));//张开右夹爪
    pick_num = 1;
    return true;
}
else 
{
   ROS_ERROR("pick_num参数错误！");
   return false;
}

return true;
}

