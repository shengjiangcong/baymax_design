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
    vector<float> res = aaa.L_xarm_forward_kinematic(mm);
    aaa.L_xarm_inverse_kinematic(res, mm);
    vector<float> nn = {-0.3199690244179359, 1.5827473502231757, -2.4085356781209604, -1.0436462959735082, 2.458701693923769, 2.225104471362355};
    vector<float> res1 = aaa.R_xarm_forward_kinematic(nn);
    aaa.R_xarm_inverse_kinematic(res1, nn);





    ros::shutdown(); 
    return 0;
}
