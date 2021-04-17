#include "moveit_cube_solver/moveit_cube_solver.h"
#include "kinematics/baymax_kinematics.h"

int main(int argc, char **argv)
{
    // 中文输出
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "moveit_cube_solver");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    //规划组
    moveit::planning_interface::MoveGroupInterface L_xarm6("L_xarm6");
    moveit::planning_interface::MoveGroupInterface R_xarm6("R_xarm6");
    moveit::planning_interface::MoveGroupInterface xarm6s("xarm6s");
    //设置运动速度
    L_xarm6.setGoalJointTolerance(0.001);
    L_xarm6.setMaxAccelerationScalingFactor(0.5);
    L_xarm6.setMaxVelocityScalingFactor(0.5);
    R_xarm6.setGoalJointTolerance(0.001);
    R_xarm6.setMaxAccelerationScalingFactor(0.5);
    R_xarm6.setMaxVelocityScalingFactor(0.5);
    xarm6s.setGoalJointTolerance(0.001);
    xarm6s.setMaxAccelerationScalingFactor(0.5);
    xarm6s.setMaxVelocityScalingFactor(0.5);

    auto l_end_effector_link = L_xarm6.getEndEffectorLink();
    auto r_end_effector_link = R_xarm6.getEndEffectorLink();
    auto xarm6s_effector_link = xarm6s.getEndEffectorLink();

    xarm6s.setNamedTarget("safe_state");
    xarm6s.move();
    sleep(1);

    geometry_msgs::Pose target_pose;
    target_pose.position.x = -0.081259;
    target_pose.position.y = 0.22535;
    target_pose.position.z = 0.2;
    target_pose.orientation.x = 0.69577;
    target_pose.orientation.y = -0.12605;
    target_pose.orientation.z = -0.12599;
    target_pose.orientation.w = 0.6958;

    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = -0.081259;
    target_pose1.position.y = -0.22535;
    target_pose1.position.z = 0.2;
    target_pose1.orientation.x = 0.69577;
    target_pose1.orientation.y = -0.12605;
    target_pose1.orientation.z = -0.12599;
    target_pose1.orientation.w = 0.6958;

    // 设置机器臂当前的状态作为运动初始状态
    L_xarm6.setStartStateToCurrentState();

    L_xarm6.setPoseTarget(target_pose);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = L_xarm6.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");  
    /*auto traj = plan.trajectory_.joint_trajectory.points;
    for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
    {
    cout << traj[i].time_from_start << "  " << traj[i].positions[0] << "  " << traj[i].velocities[0] << endl;
    }*/


    //cout << plan.planning_time_ << endl; 

    //让机械臂按照规划的轨迹开始运动。
    if(success)
      L_xarm6.execute(plan);
    sleep(1);


    moveit::planning_interface::MoveGroupInterface::Plan plan1 = plan;
    R_xarm6.setStartStateToCurrentState();
    plan1.trajectory_.joint_trajectory.joint_names[0] = "R_joint1";
    plan1.trajectory_.joint_trajectory.joint_names[1] = "R_joint2";
    plan1.trajectory_.joint_trajectory.joint_names[2] = "R_joint3";
    plan1.trajectory_.joint_trajectory.joint_names[3] = "R_joint4";
    plan1.trajectory_.joint_trajectory.joint_names[4] = "R_joint5";
    plan1.trajectory_.joint_trajectory.joint_names[5] = "R_joint6";
    BaymaxKinematics aaa;
    for (int i = 0; i < plan1.trajectory_.joint_trajectory.points.size(); i++)
    {
        vector<float> mm = {0, 0, 0, 0, 0, 0};
        for (int j = 0; j < 6; j++)
        {
            mm[j] = plan1.trajectory_.joint_trajectory.points[i].positions[j];
        }
        vector<float> res1 = aaa.R_xarm_forward_kinematic(mm);
        res1[1] = -res1[1];
        res1[3] = -res1[3];
        //res1[4] = 0.6548;
        res1[5] = -res1[5];
        vector<float> nn = {-0.0006233810964040459, -0.5366457998887636, -0.5653527320761607, 0.000455360563006252, 1.1119979487839156, 0.0005638572424650193};
        vector<float> cc = aaa.L_xarm_inverse_kinematic(res1, nn);
        //aaa.L_xarm_forward_kinematic(nn);
        for (int j = 0; j < 6; j++)
        {
            plan1.trajectory_.joint_trajectory.points[i].positions[j] = cc[j];
        }
    }

      R_xarm6.execute(plan1);

    xarm6s.setNamedTarget("safe_state");
    xarm6s.move();
    sleep(1);


    xarm6s.setStartStateToCurrentState();
    xarm6s.setPoseTarget(target_pose, l_end_effector_link);
    xarm6s.setPoseTarget(target_pose1, r_end_effector_link);

    moveit::planning_interface::MoveGroupInterface::Plan plans;
    moveit::planning_interface::MoveItErrorCode succ = xarm6s.plan(plans);

   /* for (int i = 0; i < plans.trajectory_.joint_trajectory.joint_names.size(); i++)
    {
    cout << plans.trajectory_.joint_trajectory.joint_names[i] << endl;
    }*/
    for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
    {
       /*plans.start_state_[i] = plan.start_state_[i];
       plans.start_state_[i + 6] = plan1.start_state_[i];*/
      for (int j = 0; j < 6; j++)
      {
        plans.trajectory_.joint_trajectory.points[i].positions[j] = plan.trajectory_.joint_trajectory.points[i].positions[j];
        plans.trajectory_.joint_trajectory.points[i].positions[j + 6] = plan1.trajectory_.joint_trajectory.points[i].positions[j];
      }
    }

    while (plans.trajectory_.joint_trajectory.points.size() > plan.trajectory_.joint_trajectory.points.size())
    {
         plans.trajectory_.joint_trajectory.points.pop_back();
    }
    



    xarm6s.execute(plans);

    /*ros::NodeHandle n;
    vector<float> mm = {0.5503515025905923, -0.3269694066261234, -0.7318034606615594, 0.005640544362645502, 1.0666314842561273, 0.5475066423705436};
    vector<float> res = aaa.L_xarm_forward_kinematic(mm);*/
    /*aaa.L_xarm_inverse_kinematic(res, mm);
    vector<float> nn = {-0.5632897241934713, -0.31590020037107996, -0.7418785115370501, -0.00587360798241526, 1.0664453603423867, -0.5602063095946178};
    vector<float> res1 = aaa.R_xarm_forward_kinematic(nn);
    aaa.R_xarm_inverse_kinematic(res1, nn);*/





    ros::shutdown(); 
    return 0;
}
