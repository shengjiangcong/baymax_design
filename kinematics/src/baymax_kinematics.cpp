#include <iostream>
#include "kinematics/baymax_kinematics.h"

using namespace std;
BaymaxKinematics::BaymaxKinematics()
{
    float alength[6] = { 0, 289.49, 77.58, 0, 0, 0 };//DH Table -a
    float alpha[6] = { PI / 2, 0, PI / 2, -PI / 2, PI / 2, 0 };//DH Table -alpha
    float dlength[6] = { 267, 0, 0, 342.38, 0, -123.22 };//DH Table -d
    float theta0[6] = { 0 };//DH Table -theta
    float angleOffset[6] = { 0, 0, -PI / 2, 0, -PI / 2, 0};
    float highlim[6] = { 2 * PI, 2 * PI, 2 * PI, 2 * PI, 2 * PI, 2 * PI };
    float lowlim[6] = { -2 * PI, -2 * PI, -2 * PI, -2 * PI, -2 * PI, -2 * PI };
    float JointWeight[6] = { 1, 0.6, 0.6, 0.3, 0.3, 0.3 };

    robc_ARM6DOF_set_geometry(alength, alpha, dlength, theta0);
    robc_ARM6DOF_set_angleoffset(angleOffset);
    robc_ARM6DOF_set_movelim(highlim, lowlim);
    robc_ARM6DOF_set_modle(0);
    robc_ARM6DOF_set_jointweight(JointWeight);
}

vector<float> BaymaxKinematics::L_xarm_forward_kinematic(const vector<float>& joint)
{
    float dest[6] = {0, 0, 0, 0, 0, 0};//0  -0.18623  -1.38456  0  -0.9057  0
    float posval[6] = { 0 };
    vector<float> res(6);

    for (int i = 0; i < joint.size(); i++)
    {
        dest[i] = joint[i];
    }

    dest[1] = -1 * dest[1] - 0.18623;
    dest[2] = -1 * dest[2] - 1.38456;
    dest[4] = -1 * dest[4] - 0.9057;

    robc_ARM6DOF_Anno_kicalc(dest);

    robc_ARM6DOF_updata_posval(posval);

    for (int i = 0; i < joint.size(); i++)
    {
        res[i] = posval[i];
    }

    cout << "Pos: " << res[0] << "  " << res[1] << "  " << res[2] << endl;
    cout << "Rpy: " << res[3] << "  " << res[4] << "  " << res[5] << endl;

    float tmp0 = res[0];
    float tmp1 = res[1];
    float tmp2 = res[2];
    float sum = sqrt(res[0] * res[0] + res[2] * res[2]);
    //cout << sum << endl;
    float theta = 3.141592653 / 2.0 - 0.349 + atan2(res[2], res[0]);
    //cout << theta << endl;
    res[2] = res[1];
    res[0] = -500 + sin(theta)*sum;
    res[1] = -99.16 + cos(theta)*sum;

    cout << "修正: " << res[0] << "  " << res[1] << "  " << res[2] << endl;


    Eigen::Quaternion<double> q(0,0,0,1);
    q = Eigen::AngleAxis<double>(res[5], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxis<double>(res[4], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxis<double>(res[3], ::Eigen::Vector3d::UnitX());
    //cout << "xyzw: " << q.x() << "  " << q.y() << "  " << q.z() << "  " << q.w() << endl;


    return res;
}

vector<float> BaymaxKinematics::L_xarm_inverse_kinematic(const vector<float>& pos, const vector<float>& seed)
{
    float JointValu[6] = { 0 };
    float Jointdest[6] = { 0 };
    float posdest[6] = { 0 };
    vector<float> res(6);

    for (int i = 0; i < seed.size(); i++)
    {
        JointValu[i] = seed[i];
    }

    for (int i = 0; i < pos.size(); i++)
    {
        posdest[i] = pos[i];
    }

    //posdest[1] = posdest[2];
    float sum = sqrt((500 + posdest[0]) * (500 + posdest[0]) + (99.16 + posdest[1]) * (99.16 + posdest[1]));
    //cout << sum << endl;

    float theta = 3.141592653 / 2.0 - 0.349 - atan2(500 + posdest[0], 99.16 + posdest[1]);

    posdest[1] = posdest[2];
    posdest[0] = cos(theta) * sum;
    posdest[2] = -sin(theta) * sum;
    //cout <<     posdest[0] << endl;
    //cout <<     posdest[1] << endl;
    //cout <<     posdest[2] << endl;

    //cout << theta << endl;
    robc_ARM6DOF_set_jointval(JointValu);
    robc_ARM6DOF_set_modle(0);

    robc_ARM6DOF_Anno_ikcalc(posdest);

    robc_ARM6DOF_updata_jointval(Jointdest);

    for (int i = 0; i < res.size(); i++)
    {
        res[i] = Jointdest[i];
    }
    res[1] += 0.18623;
    res[1] = -res[1];
    res[2] += 1.38456;
    res[4] += 0.9057;
    res[4] = -res[4];
    res[2] = -res[2];

    for (int i = 0; i < res.size(); i++)
    {
        while (res[i] < -1 * PI)
               res[i] += 2 * PI;
        while (res[i] > PI)
               res[i] += -2 * PI;
    }

    cout << "Joint: " << res[0] << "  " << res[1] << "  " << res[2] << "  " << res[3] << "  " << res[4] << "  " << res[5] << endl;

    return res;

}







vector<float> BaymaxKinematics::R_xarm_forward_kinematic(const vector<float>& joint)
{
    float dest[6] = {0, 0, 0, 0, 0, 0};//0  -0.18623  -1.38456  0  -0.9057  0
    float posval[6] = { 0 };
    vector<float> res(6);

    for (int i = 0; i < joint.size(); i++)
    {
        dest[i] = joint[i];
    }

    dest[1] = -1 * dest[1] - 0.18623;
    dest[2] = -1 * dest[2] - 1.38456;
    dest[4] = -1 * dest[4] - 0.9057;

    robc_ARM6DOF_Anno_kicalc(dest);

    robc_ARM6DOF_updata_posval(posval);

    for (int i = 0; i < joint.size(); i++)
    {
        res[i] = posval[i];
    }

    cout << "Pos: " << res[0] << "  " << res[1] << "  " << res[2] << endl;
    cout << "Rpy: " << res[3] << "  " << res[4] << "  " << res[5] << endl;

    float tmp0 = res[0];
    float tmp1 = res[1];
    float tmp2 = res[2];
    float sum = sqrt(res[0] * res[0] + res[2] * res[2]);
    //cout << sum << endl;
    float theta = 3.141592653 / 2.0 - 0.349 + atan2(res[2], res[0]);
    //cout << theta << endl;
    res[2] = -res[1];
    res[0] = -500 + sin(theta)*sum;
    res[1] = -(-99.16 + cos(theta)*sum);

    cout << "修正: " << res[0] << "  " << res[1] << "  " << res[2] << endl;


    Eigen::Quaternion<double> q(0,0,0,1);
    q = Eigen::AngleAxis<double>(res[5], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxis<double>(res[4], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxis<double>(res[3], ::Eigen::Vector3d::UnitX());
    //cout << "xyzw: " << q.x() << "  " << q.y() << "  " << q.z() << "  " << q.w() << endl;


    return res;
}

vector<float> BaymaxKinematics::R_xarm_inverse_kinematic(const vector<float>& pos, const vector<float>& seed)
{
    float JointValu[6] = { 0 };
    float Jointdest[6] = { 0 };
    float posdest[6] = { 0 };
    vector<float> res(6);

    for (int i = 0; i < seed.size(); i++)
    {
        JointValu[i] = seed[i];
    }

    for (int i = 0; i < pos.size(); i++)
    {
        posdest[i] = pos[i];
    }

    //posdest[1] = posdest[2];
    float sum = sqrt((500 + posdest[0]) * (500 + posdest[0]) + (99.16 - posdest[1]) * (99.16 - posdest[1]));
    //cout << sum << endl;

    float theta = 3.141592653 / 2.0 - 0.349 - atan2(500 + posdest[0], 99.16 - posdest[1]);

    posdest[1] = -posdest[2];
    posdest[0] = cos(theta) * sum;
    posdest[2] = -sin(theta) * sum;
    //cout <<     posdest[0] << endl;
    //cout <<     posdest[1] << endl;
   // cout <<     posdest[2] << endl;

    //cout << theta << endl;
    robc_ARM6DOF_set_jointval(JointValu);
    robc_ARM6DOF_set_modle(0);

    robc_ARM6DOF_Anno_ikcalc(posdest);

    robc_ARM6DOF_updata_jointval(Jointdest);

    for (int i = 0; i < res.size(); i++)
    {
        res[i] = Jointdest[i];
    }
    res[1] += 0.18623;
    res[1] = -res[1];
    res[2] += 1.38456;
    res[4] += 0.9057;
    res[4] = -res[4];
    res[2] = -res[2];

    for (int i = 0; i < res.size(); i++)
    {
        while (res[i] < -1 * PI)
               res[i] += 2 * PI;
        while (res[i] > PI)
               res[i] += -2 * PI;
    }

    cout << "Joint: " << res[0] << "  " << res[1] << "  " << res[2] << "  " << res[3] << "  " << res[4] << "  " << res[5] << endl;

    return res;

}
