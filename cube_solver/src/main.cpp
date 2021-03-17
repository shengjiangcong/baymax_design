#include "moveit_cube_solver/moveit_cube_solver.h"


int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");//中文输出

    ros::init(argc, argv, "moveit_cube_solver");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;

    CubeSolver Solver(n);//创建对象

    //Solver.remove_scene();//删除环境
    //Solver.remove_cube();//删除魔方

    Solver.move_to_safe_state();//移动到安全位置
    //Solver.take_photos();

    if(Solver.start_pick() == false)//固定位置抓取魔方，准备开始
    {
       ROS_ERROR("没有抓取起魔方，规划失败的原因");
       ros::shutdown(); 
    }

    if(Solver.call_kociemba() == false)//求解魔方
    {
       ROS_ERROR("没有求解出魔方解步骤! 输入序列错误的原因");
       ros::shutdown(); 
    }
    //std::deque<std::string> cube_deque = Solver.get_cube_deque();
    std::deque<std::string> cube_deque;

    cube_deque.push_back("L");
    cube_deque.push_back("U2");
    cube_deque.push_back("D");
    cube_deque.push_back("L'");
    cube_deque.push_back("U");
    cube_deque.push_back("L");
    cube_deque.push_back("R");
    cube_deque.push_back("B");
    cube_deque.push_back("R'");
    //cube_deque.push_back("R");
   // cube_deque.push_back("R'");
    //cube_deque.push_back("R2");
    //cube_deque.push_back("L");
    //cube_deque.push_back("B2");
    //cube_deque.push_back("R2");



    //cube_deque.push_back("L");
    //cube_deque.push_back("U");
    //cube_deque.push_back("L2");
    //cube_deque.push_back("L");
    //cube_deque.push_back("L'");
    //cube_deque.push_back("L2");
    //cube_deque.push_back("D");
    //cube_deque.push_back("R");
    //cube_deque.push_back("B");
    //cube_deque.push_back("B'");
    //cube_deque.push_back("F'");
    /*cube_deque.push_back("R");
    cube_deque.push_back("R'");
    cube_deque.push_back("R2");
cube_deque.push_back("U");*/
    while (!cube_deque.empty())
    {
        std::string a = cube_deque.front();
        //std::cout << a << std::endl;
        
        switch(Solver.solve_map[a])
        {
            case 0:
                 while (Solver.turn_U0() == false)
                 {
                    Solver.turn_U0();
                 }
                 break;
            case 1:
                 while (Solver.turn_U1() == false)
                 {
                    Solver.turn_U1();
                 }
                 break;
            case 2:
                 while (Solver.turn_U2() == false)
                 {
                    Solver.turn_U2();
                 }
                 break;
            case 3:
                 while (Solver.turn_L0() == false)
                 {
                    Solver.turn_L0();
                 }
                 break;
            case 4:
                 while (Solver.turn_L1() == false)
                 {
                    Solver.turn_L1();
                 }
                 break;
            case 5:
                 while (Solver.turn_L2() == false)
                 {
                    Solver.turn_L2();
                 }
                 break;
            case 6:
                 while (Solver.turn_D0() == false)
                 {
                    Solver.turn_D0();
                 }
                 break;
            case 7:
                 while (Solver.turn_D1() == false)
                 {
                    Solver.turn_D1();
                 }
                 break;
            case 8:
                 while (Solver.turn_D2() == false)
                 {
                    Solver.turn_D2();
                 }
                 break;
            case 9:
                 while (Solver.turn_F0() == false)
                 {
                    Solver.turn_F0();
                 }
                 break;
            case 10:
                 while (Solver.turn_F1() == false)
                 {
                    Solver.turn_F1();
                 }
                 break;
            case 11:
                 while (Solver.turn_F2() == false)
                 {
                    Solver.turn_F2();
                 }
                 break;
            case 12:
                 while (Solver.turn_R0() == false)
                 {
                    Solver.turn_R0();
                 }
                 break;
            case 13:
                 while (Solver.turn_R1() == false)
                 {
                    Solver.turn_R1();
                 }
                 break;
            case 14:
                 while (Solver.turn_R2() == false)
                 {
                    Solver.turn_R2();
                 }
                 break;
            case 15:
                 while (Solver.turn_B0() == false)
                 {
                    Solver.turn_B0();
                 }
                 break;
            case 16:
                 while (Solver.turn_B1() == false)
                 {
                    Solver.turn_B1();
                 }
                 break;
            case 17:
                 while (Solver.turn_B2() == false)
                 {
                    Solver.turn_B2();
                 }
                 break;
        
        }
        cube_deque.pop_front();
    }

    /*if (Solver.pick_num == 1)
        Solver.L_move_to_safe_state();
    else if (Solver.pick_num == 2)
        Solver.R_move_to_safe_state();*/
    

   /* geometry_msgs::Pose target_pose_l;
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

    Solver.L_xarm_move_to(target_pose_l);*/

    ros::shutdown(); 
    return 0;
}
