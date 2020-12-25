#include "moveit_cube_solver/moveit_cube_solver.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_cube_solver");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;

    CubeSolver Solver(n);

    Solver.add_scene();

    Solver.move_to_safe_state();

    if(Solver.call_kociemba() == false)
    {
       ROS_ERROR("fail to solve cube! 输入序列错误");
       ros::shutdown(); 
    }
    std::deque<std::string> cube_deque = Solver.get_cube_deque();
    while (!cube_deque.empty())
    {
        std::string a = cube_deque.front();
        std::cout << a << std::endl;
        cube_deque.pop_front();
    }
    

    geometry_msgs::Pose target_pose_l;
    target_pose_l.position.x = 0;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0;
    target_pose_l.orientation.x = 0.707;
    target_pose_l.orientation.y = 0;
    target_pose_l.orientation.z = 0;
    target_pose_l.orientation.w = 0.707;

    geometry_msgs::Pose target_pose_r;
    target_pose_r.position.x = -0.2593;
    target_pose_r.position.y = -0.13;
    target_pose_r.position.z = 0.4;
    target_pose_r.orientation.x = -0.69664;
    target_pose_r.orientation.y = -0.12282;
    target_pose_r.orientation.z = 0.12272;
    target_pose_r.orientation.w = 0.69609;

    Solver.L_xarm_move_to(target_pose_l);


    ros::shutdown(); 
    return 0;
}
