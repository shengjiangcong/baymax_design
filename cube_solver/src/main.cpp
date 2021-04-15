#include "moveit_cube_solver/moveit_cube_solver.h"
#include "kinematics/baymax_kinematics.h"

int main(int argc, char **argv)
{
    // 中文输出
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "moveit_cube_solver");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    BaymaxKinematics aaa;
    vector<float> mm = {0, 0, 0, 0, 0, 0};
    vector<float> res = aaa.forward_kinematic(mm);
    aaa.inverse_kinematic(res, mm);





    ros::shutdown(); 
    return 0;
}
