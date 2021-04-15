#ifndef _BAYMAX_KINEMATICS_H_
#define _BAYMAX_KINEMATICS_H_
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "kinematics/robcprobot_arm6DOFik.h"
#include "kinematics/robcprobot_matrix.h"

using namespace std;

class BaymaxKinematics
{
      private:
          const double PI = 3.141592653;
      public:
          BaymaxKinematics();
          vector<float> forward_kinematic(const vector<float>& joint);
          vector<float> inverse_kinematic(const vector<float>& pos, const vector<float>& seed);
          void pout() {cout << "ss" << endl;}
};

#endif

