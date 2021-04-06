#include "moveit_cube_solver/moveit_cube_solver.h"


int main(int argc, char **argv)
{
    // 中文输出
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "moveit_cube_solver");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;

    // 创建对象
    CubeSolver Solver(n);

    // 移动到安全位置
    Solver.move_to_safe_state();

    // 等待30s，用于放置魔方
    for (int i = 6; i > 0; i--)
    {
        std::cout << 5 * i << "秒后开始" << std::endl;
        sleep(5);
    }

    // 固定位置抓取魔方，准备开始
    if(Solver.start_pick() == false)
    {
       ROS_ERROR("没有抓取起魔方，规划失败");
       ros::shutdown(); 
    }

    // 拍照识别魔方
    Solver.take_photos();

    // 召唤解魔方算法服务
    if(Solver.call_kociemba() == false)
    {
       ROS_ERROR("没有求解出魔方解步骤! 输入颜色序列错误");
       ros::shutdown(); 
    }

    // 获取解
    std::deque<std::string> cube_deque = Solver.get_cube_deque();
    //std::deque<std::string> cube_deque;
    //cube_deque.push_back("U");
   
    // 循化执行，直到栈为空
    while (!cube_deque.empty())
    {
        std::string a = cube_deque.front();
        
        switch (Solver.solve_map[a])
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

    ROS_INFO("执行完成。");

    ros::shutdown(); 
    return 0;
}
