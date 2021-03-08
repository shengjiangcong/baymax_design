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
    //moveit_msgs::CollisionObject cube;
    cube.id = "cube";
    cube.header.frame_id = "ground";

    // 设置障碍物的外形、尺寸等属性   
   // shape_msgs::SolidPrimitive cube_primitive;
    cube_primitive.type = cube_primitive.BOX;
    cube_primitive.dimensions.resize(3);
    cube_primitive.dimensions[0] = 0.058;//魔方长宽高
    cube_primitive.dimensions[1] = 0.058;
    cube_primitive.dimensions[2] = 0.058;

    // 设置障碍物的位置
   // geometry_msgs::Pose cube_pose;
    cube_pose.orientation.w = 1.0;
    cube_pose.position.x = 0;//魔方位置xyz
    cube_pose.position.y = 0.05;
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
    bottom_wall_pose.position.z = -0.3;//地面的高度

    // 将障碍物的属性、位置加入到障碍物的实例中
    bottom_wall.primitives.push_back(bottom_wall_primitive);
    bottom_wall.primitive_poses.push_back(bottom_wall_pose);
    bottom_wall.operation = bottom_wall.ADD;

    // 声明一个附着物体
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "L_link6";
    attached_object.object.header.frame_id = "L_link6";
    attached_object.object.id = "box";

    // 设置附着物体的位置
    geometry_msgs::Pose attached_object_pose;
    attached_object_pose.orientation.w = 1.0;
    attached_object_pose.position.z = 0.18;
    attached_object_pose.position.y = 0.037;

    // 设置附着物体的外形、尺寸等属性   
    shape_msgs::SolidPrimitive attached_object_primitive;
    attached_object_primitive.type = attached_object_primitive.BOX;
    attached_object_primitive.dimensions.resize(3);
    attached_object_primitive.dimensions[0] = 0.01;
    attached_object_primitive.dimensions[1] = 0.01;
    attached_object_primitive.dimensions[2] = 0.04;

    attached_object.object.primitives.push_back(attached_object_primitive);
    attached_object.object.primitive_poses.push_back(attached_object_pose);
    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = std::vector<std::string>{ "L_link5", "L_link_6" };








// 声明一个附着物体
    moveit_msgs::AttachedCollisionObject attached_object1;
    attached_object1.link_name = "L_link6";
    attached_object1.object.header.frame_id = "L_link6";
    attached_object1.object.id = "box1";

    // 设置附着物体的位置
    geometry_msgs::Pose attached_object_pose1;
    attached_object_pose1.orientation.w = 1.0;
    attached_object_pose1.position.z = 0.18;
    attached_object_pose1.position.y = -0.037;

    // 设置附着物体的外形、尺寸等属性   
    shape_msgs::SolidPrimitive attached_object_primitive1;
    attached_object_primitive1.type = attached_object_primitive1.BOX;
    attached_object_primitive1.dimensions.resize(3);
    attached_object_primitive1.dimensions[0] = 0.01;
    attached_object_primitive1.dimensions[1] = 0.01;
    attached_object_primitive1.dimensions[2] = 0.04;

    attached_object1.object.primitives.push_back(attached_object_primitive1);
    attached_object1.object.primitive_poses.push_back(attached_object_pose1);
    attached_object1.object.operation = attached_object1.object.ADD;
    attached_object1.touch_links = std::vector<std::string>{ "L_link5", "L_link_6" };


// 声明一个附着物体
    moveit_msgs::AttachedCollisionObject attached_object2;
    attached_object2.link_name = "R_link6";
    attached_object2.object.header.frame_id = "R_link6";
    attached_object2.object.id = "box2";

    // 设置附着物体的位置
    geometry_msgs::Pose attached_object_pose2;
    attached_object_pose2.orientation.w = 1.0;
    attached_object_pose2.position.z = 0.18;
    attached_object_pose2.position.y = -0.037;

    // 设置附着物体的外形、尺寸等属性   
    shape_msgs::SolidPrimitive attached_object_primitive2;
    attached_object_primitive2.type = attached_object_primitive2.BOX;
    attached_object_primitive2.dimensions.resize(3);
    attached_object_primitive2.dimensions[0] = 0.01;
    attached_object_primitive2.dimensions[1] = 0.01;
    attached_object_primitive2.dimensions[2] = 0.04;

    attached_object2.object.primitives.push_back(attached_object_primitive2);
    attached_object2.object.primitive_poses.push_back(attached_object_pose2);
    attached_object2.object.operation = attached_object2.object.ADD;
    attached_object2.touch_links = std::vector<std::string>{ "R_link5", "R_link_6" };

// 声明一个附着物体
    moveit_msgs::AttachedCollisionObject attached_object3;
    attached_object3.link_name = "R_link6";
    attached_object3.object.header.frame_id = "R_link6";
    attached_object3.object.id = "box3";

    // 设置附着物体的位置
    geometry_msgs::Pose attached_object_pose3;
    attached_object_pose3.orientation.w = 1.0;
    attached_object_pose3.position.z = 0.18;
    attached_object_pose3.position.y = 0.037;

    // 设置附着物体的外形、尺寸等属性   
    shape_msgs::SolidPrimitive attached_object_primitive3;
    attached_object_primitive3.type = attached_object_primitive3.BOX;
    attached_object_primitive3.dimensions.resize(3);
    attached_object_primitive3.dimensions[0] = 0.01;
    attached_object_primitive3.dimensions[1] = 0.01;
    attached_object_primitive3.dimensions[2] = 0.04;

    attached_object3.object.primitives.push_back(attached_object_primitive3);
    attached_object3.object.primitive_poses.push_back(attached_object_pose3);
    attached_object3.object.operation = attached_object3.object.ADD;
    attached_object3.touch_links = std::vector<std::string>{ "R_link5", "R_link_6" };
















        // 所有障碍物加入列表后，再把障碍物加入到当前的情景中
    planning_scene.world.collision_objects.push_back(bottom_wall);
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.world.collision_objects.push_back(attached_object1.object);
    planning_scene.world.collision_objects.push_back(attached_object2.object);
    planning_scene.world.collision_objects.push_back(attached_object3.object);
    //planning_scene.world.collision_objects.push_back(cube);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    // 声明去附着属性的物体
    moveit_msgs::CollisionObject remove_box_object;
    remove_box_object.id = "box";
    remove_box_object.header.frame_id = "L_link6";
    remove_box_object.operation = remove_box_object.REMOVE;

    // 将物体附着到机器人上，并从场景中删除
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_box_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_diff_publisher.publish(planning_scene);












    // 声明去附着属性的物体
    moveit_msgs::CollisionObject remove_box_object1;
    remove_box_object1.id = "box1";
    remove_box_object1.header.frame_id = "L_link6";
    remove_box_object1.operation = remove_box_object1.REMOVE;

    // 将物体附着到机器人上，并从场景中删除
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_box_object1);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object1);
    planning_scene_diff_publisher.publish(planning_scene);


    // 声明去附着属性的物体
    moveit_msgs::CollisionObject remove_box_object2;
    remove_box_object2.id = "box2";
    remove_box_object2.header.frame_id = "R_link6";
    remove_box_object2.operation = remove_box_object2.REMOVE;

    // 将物体附着到机器人上，并从场景中删除
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_box_object2);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object2);
    planning_scene_diff_publisher.publish(planning_scene);

    // 声明去附着属性的物体
    moveit_msgs::CollisionObject remove_box_object3;
    remove_box_object3.id = "box3";
    remove_box_object3.header.frame_id = "R_link6";
    remove_box_object3.operation = remove_box_object3.REMOVE;

    // 将物体附着到机器人上，并从场景中删除
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_box_object3);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object3);
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

void CubeSolver::R_move_to_ready_state()
{
    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0.0;
    target_pose_r.position.y = -0.2;
    target_pose_r.position.z = 0;
    target_pose_r.orientation.x = -0.5;
    target_pose_r.orientation.y = -0.5;
    target_pose_r.orientation.z = -0.5;
    target_pose_r.orientation.w = 0.5;
    R_xarm_move_to(target_pose_r);
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

void CubeSolver::remove_cube()//模型删除魔方
{
    moveit_msgs::CollisionObject remove_cylinder_object;
    remove_cylinder_object.id = "cube";
    remove_cylinder_object.header.frame_id = "ground";
    remove_cylinder_object.operation = remove_cylinder_object.REMOVE;
    planning_scene.world.collision_objects.push_back(remove_cylinder_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
}

void CubeSolver::add_cube()//模型添加魔方
{
    planning_scene.world.collision_objects.push_back(cube);
    planning_scene.is_diff = true;
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
    remove_cube();
    std::vector<double> joint_group_positions = L_xarm.getCurrentJointValues();
    joint_group_positions[index] += kind * PI / 2.0;
    L_xarm.setJointValueTarget(joint_group_positions);
    L_xarm.move();
    add_cube();
}

bool CubeSolver::R_xarm_move_to(int index, double kind)
{
    remove_cube();
    std::vector<double> joint_group_positions = R_xarm.getCurrentJointValues();
    joint_group_positions[index] += kind * PI / 2.0;
    R_xarm.setJointValueTarget(joint_group_positions);
    R_xarm.move();
    add_cube();
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

    Gripper_mode mode = R_open;
    gripper_control(mode);

    target_pose_r.position.z = -0.12;
   
    if(R_xarm_move_to(target_pose_r) == false)
       return false;

    mode = R_closed;
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

void CubeSolver::gripper_control_test()
{

    ros::ServiceClient l_client = nh_.serviceClient<xarm_msgs::GripperMove>("L_xarm6/gripper_move");
    ros::ServiceClient r_client = nh_.serviceClient<xarm_msgs::GripperMove>("R_xarm6/gripper_move");

    xarm_msgs::GripperMove l_srv;
    xarm_msgs::GripperMove r_srv;
    l_srv.request.pulse_pos = 850;
    r_srv.request.pulse_pos = 300;
    l_client.call(l_srv);
    r_client.call(r_srv);
}

void CubeSolver::gripper_control(Gripper_mode mode)
{

    ros::ServiceClient l_client = nh_.serviceClient<xarm_msgs::GripperMove>("L_xarm6/gripper_move");
    ros::ServiceClient r_client = nh_.serviceClient<xarm_msgs::GripperMove>("R_xarm6/gripper_move");

    xarm_msgs::GripperMove l_srv;
    xarm_msgs::GripperMove r_srv;
    //l_srv.request.pulse_pos = 700;
    //l_client.call(l_srv);
if (mode == L_closed)
{
    l_srv.request.pulse_pos = 570;
    l_client.call(l_srv);
    ROS_INFO("左手夹爪闭合");
}
else if (mode == L_open)
{
    l_srv.request.pulse_pos = 850;
    l_client.call(l_srv);
    ROS_INFO("左手夹爪张开");
}
else if (mode == R_closed)
{
    r_srv.request.pulse_pos = 570;
    r_client.call(r_srv);
    ROS_INFO("右手夹爪闭合");
}
else if (mode == R_open)
{
    r_srv.request.pulse_pos = 850;
    r_client.call(r_srv);
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
    add_cube();
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.05;
    target_pose_l.position.z = 0.225;
    target_pose_l.orientation.x = 1;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;

    /*target_pose_l.position.z = 0.245;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;*/

    remove_cube();

    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,1);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    add_cube();

    target_pose_l.position.z = 0.3;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.05;
    target_pose_l.position.z = 0.225;
    target_pose_l.orientation.x = 1;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,-1);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪
    add_cube();

    target_pose_l.position.z = 0.3;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.05;
    target_pose_l.position.z = 0.225;
    target_pose_l.orientation.x = 1;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,2);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪
    add_cube();

    target_pose_l.position.z = 0.3;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.275;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = -0.5;
    target_pose_l.orientation.y = -0.5;
    target_pose_l.orientation.z = 0.5;
    target_pose_l.orientation.w = -0.5;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
    /*target_pose_l.position.y = 0.275;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;*/

    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,1);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    add_cube();

    target_pose_l.position.y = 0.35;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.275;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = -0.5;
    target_pose_l.orientation.y = -0.5;
    target_pose_l.orientation.z = 0.5;
    target_pose_l.orientation.w = -0.5;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;

    /*target_pose_l.position.y = 0.275;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;*/

    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,-1);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪
    add_cube();

    target_pose_l.position.y = 0.35;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;

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
    add_cube();
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.275;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = -0.5;
    target_pose_l.orientation.y = -0.5;
    target_pose_l.orientation.z = 0.5;
    target_pose_l.orientation.w = -0.5;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;

    /*target_pose_l.position.y = 0.275;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;*/

    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,2);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪

    add_cube();

    target_pose_l.position.y = 0.35;
    if(L_xarm_move_to(target_pose_l) == false)
       return false;
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

    R_xarm_move_to(5,2);

    turn_U0();


    R_xarm_move_to(5,-2);

    /*add_cube();
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
    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,1);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪
    add_cube();*/
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
    //remove_cube();

    R_xarm_move_to(5,2);

    turn_U1();

    R_xarm_move_to(5,-2);
    /*add_cube();
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
    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,-1);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪
    add_cube();*/
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

    //remove_cube();

    R_xarm_move_to(5,2);

    turn_U2();

    R_xarm_move_to(5,-2);
    /*add_cube();
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
    remove_cube();
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    L_xarm_move_to(5,2);
    gripper_control((Gripper_mode)(L_open));//张开左夹爪
    add_cube();*/
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

    L_xarm_move_to(5, -2);

    turn_B0();

    L_xarm_move_to(5, 2);
/*gripper_control((Gripper_mode)(R_open));//张开左夹爪

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
gripper_control((Gripper_mode)(R_open));//张开左夹爪*/
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

    L_xarm_move_to(5, -2);

    turn_B1();

    L_xarm_move_to(5, 2);
/*gripper_control((Gripper_mode)(R_open));//张开左夹爪

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
gripper_control((Gripper_mode)(R_open));//张开左夹爪*/
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

    L_xarm_move_to(5, -2);

    turn_B2();

    L_xarm_move_to(5, 2);
/*gripper_control((Gripper_mode)(R_open));//张开左夹爪

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
gripper_control((Gripper_mode)(R_open));//张开左夹爪*/
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
    add_cube();
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = -0.165;
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
    target_pose_r.position.y = -0.25;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = -0.165;
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
    target_pose_r.position.y = -0.25;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = -0.165;
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
    target_pose_r.position.y = -0.25;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0.05;
    target_pose_r.position.z = 0.225;
    target_pose_r.orientation.x = 0.7071068;
    target_pose_r.orientation.y = 0.7071068;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
    gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
    R_xarm_move_to(5,1);
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    target_pose_r.position.z = 0.28;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0.05;
    target_pose_r.position.z = 0.225;
    target_pose_r.orientation.x = 0.7071068;
    target_pose_r.orientation.y = 0.7071068;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
    gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
    R_xarm_move_to(5, -1);
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    target_pose_r.position.z = 0.28;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
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
    add_cube();
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = 0;
    target_pose_r.position.y = 0.05;
    target_pose_r.position.z = 0.225;
    target_pose_r.orientation.x = 0.7071068;
    target_pose_r.orientation.y = 0.7071068;
    target_pose_r.orientation.z = 0;
    target_pose_r.orientation.w = 0;

    if(R_xarm_move_to(target_pose_r) == false)
       return false;
    gripper_control((Gripper_mode)(R_closed));//闭合左夹爪
    R_xarm_move_to(5, 2);
    gripper_control((Gripper_mode)(R_open));//张开左夹爪

    target_pose_r.position.z = 0.28;
    if(R_xarm_move_to(target_pose_r) == false)
       return false;
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

    add_cube();

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.25;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = 0.7071068;
    target_pose_l.orientation.y = 0;
    target_pose_l.orientation.z = 0;
    target_pose_l.orientation.w = 0.7071068;

    if(L_xarm_move_to(target_pose_l) == false)
       return false;
    gripper_control((Gripper_mode)(L_closed));//闭合左夹爪
    gripper_control((Gripper_mode)(R_open));//张开右夹爪
    R_move_to_ready_state();
    L_xarm_move_to(5, 1);
    pick_num = 2;
    return true;
}
else if (pick_num == 2)
{    
    gripper_control((Gripper_mode)(R_open));//张开右夹爪

    add_cube();
    R_move_to_ready_state();
    L_xarm_move_to(5, -1);

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

    L_move_to_safe_state();
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

