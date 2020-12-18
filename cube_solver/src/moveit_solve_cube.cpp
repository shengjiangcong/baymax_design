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
#include "moveit_solve_cube.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_solve_cube");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("xarm6s");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "ground";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    // 设置机器人终端的目标位置L
    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = -0.2593;
    target_pose_l.position.y = 0.13;
    target_pose_l.position.z = 0.2;
    target_pose_l.orientation.x = 0.69664;
    target_pose_l.orientation.y = -0.12282;
    target_pose_l.orientation.z = -0.12272;
    target_pose_l.orientation.w = 0.69609;
    // 设置机器人终端的目标位置R
    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = -0.2593;
    target_pose_r.position.y = -0.13;
    target_pose_r.position.z = 0.4;
    target_pose_r.orientation.x = -0.69664;
    target_pose_r.orientation.y = -0.12282;
    target_pose_r.orientation.z = 0.12272;
    target_pose_r.orientation.w = 0.69609;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose_l,"L_link6");
    arm.setPoseTarget(target_pose_r,"R_link6");

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    //让机械臂按照规划的轨迹开始运动。
    if(success)
      arm.execute(plan);
    sleep(1);


    ros::shutdown(); 

    return 0;
}
