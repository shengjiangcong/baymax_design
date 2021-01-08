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
    xarms("xarm6s")
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
    cube_pose.position.z = -0.25;

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
    bottom_wall_pose.position.z = -0.3;//地面的高度

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
}

bool CubeSolver::turn_U0()
{
    ROS_INFO("顺时针旋转上面90度。");
return true;
}

bool CubeSolver::turn_U1()
{
    ROS_INFO("逆时针旋转上面90度。");
return true;
}

bool CubeSolver::turn_U2()
{
    ROS_INFO("旋转上面180度。");
return true;
}

bool CubeSolver::turn_L0()
{
    ROS_INFO("顺时针旋转左面90度。");
return true;
}

bool CubeSolver::turn_L1()
{
    ROS_INFO("逆时针旋转左面90度。");
return true;
}

bool CubeSolver::turn_L2()
{
    ROS_INFO("旋转左面180度。");
return true;
}

bool CubeSolver::turn_D0()
{
    ROS_INFO("顺时针旋转下面90度。");
return true;
}

bool CubeSolver::turn_D1()
{
    ROS_INFO("逆时针旋转下面90度。");
return true;
}

bool CubeSolver::turn_D2()
{
    ROS_INFO("旋转下面180度。");
return true;
}

bool CubeSolver::turn_F0()
{
    ROS_INFO("顺时针旋转前面90度。");
return true;
}

bool CubeSolver::turn_F1()
{
    ROS_INFO("逆时针旋转前面90度。");
return true;
}

bool CubeSolver::turn_F2()
{
    ROS_INFO("旋转前面180度。");
return true;
}

bool CubeSolver::turn_R0()
{
    ROS_INFO("顺时针旋转右面90度。");
return true;
}

bool CubeSolver::turn_R1()
{
    ROS_INFO("逆时针旋转右面90度。");
return true;
}

bool CubeSolver::turn_R2()
{
    ROS_INFO("旋转右面180度。");
return true;
}bool CubeSolver::turn_B0()
{
    ROS_INFO("顺时针旋转后面90度。");
return true;
}

bool CubeSolver::turn_B1()
{
    ROS_INFO("逆时针旋转后面90度。");
return true;
}

bool CubeSolver::turn_B2()
{
    ROS_INFO("旋转后面180度。");
return true;
}

