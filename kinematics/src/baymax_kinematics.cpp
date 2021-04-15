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

vector<float> BaymaxKinematics::forward_kinematic(const vector<float>& joint)
{
    float dest[6] = { -0.65, 1.56, -1.56, 1.05, 0.34, 1.83};//0  -0.18623  -1.38456  0  -0.9057  0
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

    return res;
}

vector<float> BaymaxKinematics::inverse_kinematic(const vector<float>& pos, const vector<float>& seed)
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

    robc_ARM6DOF_set_jointval(JointValu);
    robc_ARM6DOF_set_modle(0);

    robc_ARM6DOF_Anno_ikcalc(posdest);

    robc_ARM6DOF_updata_jointval(Jointdest);

    for (int i = 0; i < res.size(); i++)
    {
        res[i] = Jointdest[i];
    }

    cout << "Joint: " << res[0] << "  " << res[1] << "  " << res[2] << "  " << res[3] << "  " << res[4] << "  " << res[5] << endl;

    return res;

}
