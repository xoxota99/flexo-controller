/*
  ik_solver.cpp - Inverse / Forward kinematics solver
  Part of flexo-controller

  Copyright (c) 2019 Phil Desrosiers

  Flexo is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Flexo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Flexo.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ik_solver.h"

const ik_config_t dh_params[] = {
    {PI / 2, 140.25, 90.0},
    {0.0, 0.0, 250.0},
    {PI / 2, 0.0, 66.125},
    {PI / 2, 342.451, 0.0},
    {PI / 2, 0.0, 0.0},
    {PI, 0.0, 61.350}};

const double sin_alpha[] = {
    sin(dh_params[0].alpha),
    sin(dh_params[1].alpha),
    sin(dh_params[2].alpha),
    sin(dh_params[3].alpha),
    sin(dh_params[4].alpha),
    sin(dh_params[5].alpha)};

const double cos_alpha[] = {
    cos(dh_params[0].alpha),
    cos(dh_params[1].alpha),
    cos(dh_params[2].alpha),
    cos(dh_params[3].alpha),
    cos(dh_params[4].alpha),
    cos(dh_params[5].alpha)};

const frame_t world_frame = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //angles in degrees
const frame_t tool_frame = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //angles in degrees

const double cos_wf_roll = cos(radians(world_frame.roll));
const double cos_wf_pitch = cos(radians(world_frame.pitch));
const double cos_wf_yaw = cos(radians(world_frame.yaw));
const double sin_wf_roll = sin(radians(world_frame.roll));
const double sin_wf_pitch = sin(radians(world_frame.pitch));
const double sin_wf_yaw = sin(radians(world_frame.yaw));

const double _wf[3][4] = {
    {cos_wf_roll * cos_wf_pitch, -sin_wf_roll *cos_wf_yaw + cos_wf_roll *sin_wf_pitch *sin_wf_yaw, sin_wf_roll *sin_wf_yaw + cos_wf_roll *sin_wf_pitch *cos_wf_yaw, world_frame.x},
    {sin_wf_roll * cos_wf_pitch, cos_wf_roll *cos_wf_yaw + sin_wf_roll *sin_wf_pitch *sin_wf_yaw, -cos_wf_roll *sin_wf_yaw + sin_wf_roll *sin_wf_pitch *cos_wf_yaw, world_frame.y},
    {-sin_wf_roll, cos_wf_pitch *sin_wf_yaw, cos_wf_pitch *cos_wf_yaw, world_frame.z},
}; //{0, 0, 0, 1}};

const double cos_tf_roll = cos(radians(tool_frame.roll));
const double cos_tf_pitch = cos(radians(tool_frame.pitch));
const double cos_tf_yaw = cos(radians(tool_frame.yaw));
const double sin_tf_roll = sin(radians(tool_frame.roll));
const double sin_tf_pitch = sin(radians(tool_frame.pitch));
const double sin_tf_yaw = sin(radians(tool_frame.yaw));

const double _tf[3][4] = {
    {cos_tf_roll * cos_tf_pitch, -sin_tf_roll *cos_tf_yaw + cos_tf_roll *sin_tf_pitch *sin_tf_yaw, sin_tf_roll *sin_tf_yaw + cos_tf_roll *sin_tf_pitch *cos_tf_yaw, world_frame.x},
    {sin_tf_roll * cos_tf_pitch, cos_tf_roll *cos_tf_yaw + sin_tf_roll *sin_tf_pitch *sin_tf_yaw, -cos_tf_roll *sin_tf_yaw + sin_tf_roll *sin_tf_pitch *cos_tf_yaw, world_frame.y},
    {-sin_tf_roll, cos_tf_pitch *sin_tf_yaw, cos_tf_pitch *cos_tf_yaw, world_frame.z},
}; //{0, 0, 0, 1}};

void inverse(double *solution, const double x_pos, const double y_pos, const double z_pos, const double yaw, const double pitch, const double roll)
{
    static const double _r_r_0_6[3][4] = {
        {-1, sin(radians(180)), 0, 0},
        {-sin(radians(180)) * cos_alpha[5], -cos_alpha[5], sin_alpha[5], 0},
        {sin(radians(180)) * sin_alpha[5], sin_alpha[5], cos_alpha[5], -dh_params[5].d},
    }; //{0, 0, 0, 1}};

    const double cos_yaw = cos(radians(yaw));
    const double cos_roll = cos(radians(roll));
    const double cos_pitch = cos(radians(pitch));
    const double sin_yaw = sin(radians(yaw));
    const double sin_roll = sin(radians(roll));
    const double sin_pitch = sin(radians(pitch));

    const double r_0_t[3][4] = {
        {cos_yaw * cos_roll - cos_pitch * sin_yaw * sin_roll, cos_roll * sin_yaw + cos_yaw * cos_pitch * sin_roll, sin_pitch * sin_roll, x_pos},
        {cos_pitch * cos_roll * sin_yaw + cos_yaw * sin_roll, cos_yaw * cos_pitch * cos_roll - sin_yaw * sin_roll, cos_roll * sin_pitch, y_pos},
        {sin_yaw * sin_pitch, cos_yaw * sin_pitch, -cos_pitch, z_pos},
    }; //{0, 0, 0, 1}};

    const double r_0_t_off[3][4] = {
        {-((_wf[0][0] * r_0_t[0][0]) + (_wf[0][1] * r_0_t[1][0]) + (_wf[0][2] * r_0_t[2][0])), (_wf[0][0] * r_0_t[0][1]) + (_wf[0][1] * r_0_t[1][1]) + (_wf[0][2] * r_0_t[2][1]), (_wf[0][0] * r_0_t[0][2]) + (_wf[0][1] * r_0_t[1][2]) + (_wf[0][2] * r_0_t[2][2]), (_wf[0][0] * r_0_t[0][3]) + (_wf[0][1] * r_0_t[1][3]) + (_wf[0][2] * r_0_t[2][3]) + _wf[0][3]},
        {(_wf[1][0] * r_0_t[0][0]) + (_wf[1][1] * r_0_t[1][0]) + (_wf[1][2] * r_0_t[2][0]), (_wf[1][0] * r_0_t[0][1]) + (_wf[1][1] * r_0_t[1][1]) + (_wf[1][2] * r_0_t[2][1]), (_wf[1][0] * r_0_t[0][2]) + (_wf[1][1] * r_0_t[1][2]) + (_wf[1][2] * r_0_t[2][2]), (_wf[1][0] * r_0_t[0][3]) + (_wf[1][1] * r_0_t[1][3]) + (_wf[1][2] * r_0_t[2][3]) + _wf[1][3]},
        {(_wf[2][0] * r_0_t[0][0]) + (_wf[2][1] * r_0_t[1][0]) + (_wf[2][2] * r_0_t[2][0]), (_wf[2][0] * r_0_t[0][1]) + (_wf[2][1] * r_0_t[1][1]) + (_wf[2][2] * r_0_t[2][1]), (_wf[2][0] * r_0_t[0][2]) + (_wf[2][1] * r_0_t[1][2]) + (_wf[2][2] * r_0_t[2][2]), (_wf[2][0] * r_0_t[0][3]) + (_wf[2][1] * r_0_t[1][3]) + (_wf[2][2] * r_0_t[2][3]) + _wf[2][3]},
    }; //{0, 0, 0, 1}};

    const double r_0_6[4][4] = {
        {(r_0_t_off[0][0] * _tf[0][0]) + (r_0_t_off[0][1] * _tf[0][1]) + (r_0_t_off[0][2] * _tf[0][2]) + (r_0_t_off[0][3] * tool_frame.x), (r_0_t_off[0][0] * _tf[1][0]) + (r_0_t_off[0][1] * _tf[1][1]) + (r_0_t_off[0][2] * _tf[1][2]) + (r_0_t_off[0][3] * tool_frame.y), (r_0_t_off[0][0] * _tf[2][0]) + (r_0_t_off[0][1] * _tf[2][1]) + (r_0_t_off[0][2] * _tf[2][2]) + (r_0_t_off[0][3] * tool_frame.z), r_0_t_off[0][3]},
        {(r_0_t_off[1][0] * _tf[0][0]) + (r_0_t_off[1][1] * _tf[0][1]) + (r_0_t_off[1][2] * _tf[0][2]) + (r_0_t_off[1][3] * tool_frame.x), (r_0_t_off[1][0] * _tf[1][0]) + (r_0_t_off[1][1] * _tf[1][1]) + (r_0_t_off[1][2] * _tf[1][2]) + (r_0_t_off[1][3] * tool_frame.y), (r_0_t_off[1][0] * _tf[2][0]) + (r_0_t_off[1][1] * _tf[2][1]) + (r_0_t_off[1][2] * _tf[2][2]) + (r_0_t_off[1][3] * tool_frame.z), r_0_t_off[1][3]},
        {(r_0_t_off[2][0] * _tf[0][0]) + (r_0_t_off[2][1] * _tf[0][1]) + (r_0_t_off[2][2] * _tf[0][2]) + (r_0_t_off[2][3] * tool_frame.x), (r_0_t_off[2][0] * _tf[1][0]) + (r_0_t_off[2][1] * _tf[1][1]) + (r_0_t_off[2][2] * _tf[1][2]) + (r_0_t_off[2][3] * tool_frame.y), (r_0_t_off[2][0] * _tf[2][0]) + (r_0_t_off[2][1] * _tf[2][1]) + (r_0_t_off[2][2] * _tf[2][2]) + (r_0_t_off[2][3] * tool_frame.z), r_0_t_off[2][3]},
        {tool_frame.x, tool_frame.y, tool_frame.z, 1}};

    const double r_0_5[4][4] = {
        {(-r_0_6[0][0]) + (r_0_6[0][1] * _r_r_0_6[1][0]) + (r_0_6[0][2] * _r_r_0_6[2][0]), (r_0_6[0][0] * _r_r_0_6[0][1]) + (r_0_6[0][1] * _r_r_0_6[1][1]) + (r_0_6[0][2] * _r_r_0_6[2][1]), (r_0_6[0][1] * _r_r_0_6[1][2]) + (r_0_6[0][2] * _r_r_0_6[2][2]), (r_0_6[0][1] * _r_r_0_6[1][3]) + (r_0_6[0][2] * _r_r_0_6[2][3]) + r_0_6[0][3]},
        {(-r_0_6[1][0]) + (r_0_6[1][1] * _r_r_0_6[1][0]) + (r_0_6[1][2] * _r_r_0_6[2][0]), (r_0_6[1][0] * _r_r_0_6[0][1]) + (r_0_6[1][1] * _r_r_0_6[1][1]) + (r_0_6[1][2] * _r_r_0_6[2][1]), (r_0_6[1][1] * _r_r_0_6[1][2]) + (r_0_6[1][2] * _r_r_0_6[2][2]), (r_0_6[1][1] * _r_r_0_6[1][3]) + (r_0_6[1][2] * _r_r_0_6[2][3]) + r_0_6[1][3]},
        {(-r_0_6[2][0]) + (r_0_6[2][1] * _r_r_0_6[1][0]) + (r_0_6[2][2] * _r_r_0_6[2][0]), (r_0_6[2][0] * _r_r_0_6[0][1]) + (r_0_6[2][1] * _r_r_0_6[1][1]) + (r_0_6[2][2] * _r_r_0_6[2][1]), (r_0_6[2][1] * _r_r_0_6[1][2]) + (r_0_6[2][2] * _r_r_0_6[2][2]), (r_0_6[2][1] * _r_r_0_6[1][3]) + (r_0_6[2][2] * _r_r_0_6[2][3]) + r_0_6[2][3]},
        {(-tool_frame.x) + (tool_frame.y * _r_r_0_6[1][0]) + (tool_frame.z * _r_r_0_6[2][0]), (tool_frame.x * _r_r_0_6[0][1]) + (tool_frame.y * _r_r_0_6[1][1]) + (tool_frame.z * _r_r_0_6[2][1]), (tool_frame.y * _r_r_0_6[1][2]) + (tool_frame.z * _r_r_0_6[2][2]), (tool_frame.y * _r_r_0_6[1][3]) + (tool_frame.z * _r_r_0_6[2][3]) + 1}};

    const double pX[2] = {sqrt(sq(r_0_5[1][3]) + sq(r_0_5[0][3])), sqrt(sq(r_0_5[1][3]) + sq(r_0_5[0][3]))};
    const double pY[2] = {r_0_5[2][3] - dh_params[0].d, r_0_5[2][3] - dh_params[0].d};
    const double pX_a1[2] = {pX[0] - dh_params[0].a, -(pX[0] - dh_params[0].a)};
    const double pa2H[2] = {sqrt((sq(pY[0])) + sq(pX_a1[0])), sqrt((sq(pY[0])) + sq(pX_a1[1]))};
    static const double _pa3H[2] = {sqrt((sq(dh_params[3].d)) + sq(dh_params[2].a)), sqrt((sq(dh_params[3].d)) + sq(dh_params[2].a))};
    const double theta_A[2] = {degrees(atan(pY[0] / pX_a1[0])), degrees(acos(((sq(dh_params[1].a)) + sq(pa2H[1]) - sq(dh_params[3].d)) / (2 * dh_params[1].a * pa2H[1])))};
    const double theta_B[2] = {degrees(acos(((sq(dh_params[1].a)) + sq(pa2H[0]) - sq(_pa3H[0])) / (2 * dh_params[1].a * pa2H[0]))), degrees(atan(pX_a1[1] / pY[0]))};
    static const double _theta_E[2] = {degrees(atan(abs(dh_params[3].d) / dh_params[2].a)), degrees(atan(abs(dh_params[3].d) / dh_params[2].a))};
    const double theta_C[2] = {180 - degrees(acos(((sq(_pa3H[0])) + sq(dh_params[1].a) - sq(pa2H[0])) / (2 * abs(_pa3H[0]) * dh_params[1].a))) + (90 - _theta_E[0]), 180 - degrees(acos(((sq(_pa3H[0])) + sq(dh_params[1].a) - sq(pa2H[1])) / (2 * abs(_pa3H[0]) * dh_params[1].a))) + (90 - _theta_E[0])};
    const double theta_D[2] = {0, 90 - (theta_A[1] + theta_B[1])};
    const double J2_theta[2] = {-(theta_A[0] + theta_B[0]), theta_D[1] - 180};

    // J1
    double j1deg = degrees(atan((r_0_5[1][3]) / r_0_5[0][3]));
    const int quadrant = (x_pos > 0 ? (y_pos >= 0 ? 0 : 1) : (y_pos >= 0 ? 3 : 2));

    switch (quadrant)
    {
    case 2:
        j1deg -= 180;
        break;
    case 3:
        j1deg += 180;
    // case 0:
    // case 1:
    default:
        break;
    };

    solution[0] = j1deg;

    const double j1[3][4] = {
        {cos(radians(j1deg)), -sin(radians(j1deg)) * cos_alpha[0], sin(radians(j1deg)) * sin_alpha[0], dh_params[0].a * cos(radians(j1deg))},
        {sin(radians(j1deg)), cos(radians(j1deg)) * cos_alpha[0], -cos(radians(j1deg)) * sin_alpha[0], dh_params[0].a * sin(radians(j1deg))},
        {0, sin_alpha[0], cos_alpha[0], dh_params[0].d},
    }; //{0, 0, 0, 1}};

    const double r_0_1[3][4] = {
        {(_wf[0][0] * j1[0][0]) + (_wf[0][1] * j1[1][0]), (_wf[0][0] * j1[0][1]) + (_wf[0][1] * j1[1][1]) + (_wf[0][2] * j1[2][1]), (_wf[0][0] * j1[0][2]) + (_wf[0][1] * j1[1][2]) + (_wf[0][2] * j1[2][2]), (_wf[0][0] * j1[0][3]) + (_wf[0][1] * j1[1][3]) + (_wf[0][2] * j1[2][3]) + _wf[0][3]},
        {(_wf[1][0] * j1[0][0]) + (_wf[1][1] * j1[1][0]), (_wf[1][0] * j1[0][1]) + (_wf[1][1] * j1[1][1]) + (_wf[1][2] * j1[2][1]), (_wf[1][0] * j1[0][2]) + (_wf[1][1] * j1[1][2]) + (_wf[1][2] * j1[2][2]), (_wf[1][0] * j1[0][3]) + (_wf[1][1] * j1[1][3]) + (_wf[1][2] * j1[2][3]) + _wf[1][3]},
        {(_wf[2][0] * j1[0][0]) + (_wf[2][1] * j1[1][0]), (_wf[2][0] * j1[0][1]) + (_wf[2][1] * j1[1][1]) + (_wf[2][2] * j1[2][1]), (_wf[2][0] * j1[0][2]) + (_wf[2][1] * j1[1][2]) + (_wf[2][2] * j1[2][2]), (_wf[2][0] * j1[0][3]) + (_wf[2][1] * j1[1][3]) + (_wf[2][2] * j1[2][3]) + _wf[2][3]},
    }; //{0, 0, 0, 1}};

    // J2
    solution[1] = J2_theta[pX_a1[0] >= 0]; //(pX_a1[0] < 0 ? J2_theta[0] : J2_theta[1]);

    const double j2[3][4] = {
        {cos(radians(solution[1])), -sin(radians(solution[1])) * cos_alpha[1], sin(radians(solution[1])) * sin_alpha[1], dh_params[1].a * cos(radians(solution[1]))},
        {sin(radians(solution[1])), cos(radians(solution[1])) * cos_alpha[1], -cos(radians(solution[1])) * sin_alpha[1], dh_params[1].a * sin(radians(solution[1]))},
        {0, sin_alpha[1], cos_alpha[1], dh_params[1].d},
    }; //{0, 0, 0, 1}};

    // J3
    solution[2] = theta_C[pX_a1[0] >= 0]; //(pX_a1[0] < 0 ? theta_C[0] : theta_C[1]);
    const double j3[3][4] = {
        {cos(radians((solution[2]) - 90)), -sin(radians((solution[2]) - 90)) * cos_alpha[2], sin(radians((solution[2]) - 90)) * sin_alpha[2], dh_params[2].a * cos(radians((solution[2]) - 90))},
        {sin(radians((solution[2]) - 90)), cos(radians((solution[2]) - 90)) * cos_alpha[2], -cos(radians((solution[2]) - 90)) * sin_alpha[2], dh_params[2].a * sin(radians((solution[2]) - 90))},
        {0, sin_alpha[2], cos_alpha[2], dh_params[2].d},
    }; //{0, 0, 0, 1}};

    const double r_0_2[4][4] = {
        {(r_0_1[0][0] * j2[0][0]) + (r_0_1[0][1] * j2[1][0]) + (r_0_1[0][2] * j2[2][0]), (r_0_1[0][0] * j2[0][1]) + (r_0_1[0][1] * j2[1][1]) + (r_0_1[0][2] * j2[2][1]), (r_0_1[0][0] * j2[0][2]) + (r_0_1[0][1] * j2[1][2]) + (r_0_1[0][2] * j2[2][2]), (r_0_1[0][0] * j2[0][3]) + (r_0_1[0][1] * j2[1][3]) + (r_0_1[0][2] * j2[2][3]) + r_0_1[0][3]},
        {(r_0_1[1][0] * j2[0][0]) + (r_0_1[1][1] * j2[1][0]) + (r_0_1[1][2] * j2[2][0]), (r_0_1[1][0] * j2[0][1]) + (r_0_1[1][1] * j2[1][1]) + (r_0_1[1][2] * j2[2][1]), (r_0_1[1][0] * j2[0][2]) + (r_0_1[1][1] * j2[1][2]) + (r_0_1[1][2] * j2[2][2]), (r_0_1[1][0] * j2[0][3]) + (r_0_1[1][1] * j2[1][3]) + (r_0_1[1][2] * j2[2][3]) + r_0_1[1][3]},
        {(r_0_1[2][0] * j2[0][0]) + (r_0_1[2][1] * j2[1][0]) + (r_0_1[2][2] * j2[2][0]), (r_0_1[2][0] * j2[0][1]) + (r_0_1[2][1] * j2[1][1]) + (r_0_1[2][2] * j2[2][1]), (r_0_1[2][0] * j2[0][2]) + (r_0_1[2][1] * j2[1][2]) + (r_0_1[2][2] * j2[2][2]), (r_0_1[2][0] * j2[0][3]) + (r_0_1[2][1] * j2[1][3]) + (r_0_1[2][2] * j2[2][3]) + r_0_1[2][3]},
    }; //{0, 0, 0, 1}};

    const double r_0_3[3][4] = {
        {(r_0_2[0][0] * j3[0][0]) + (r_0_2[0][1] * j3[1][0]) + (r_0_2[0][2] * j3[2][0]), (r_0_2[0][0] * j3[0][1]) + (r_0_2[0][1] * j3[1][1]) + (r_0_2[0][2] * j3[2][1]), (r_0_2[0][0] * j3[0][2]) + (r_0_2[0][1] * j3[1][2]) + (r_0_2[0][2] * j3[2][2]), (r_0_2[0][0] * j3[0][3]) + (r_0_2[0][1] * j3[1][3]) + (r_0_2[0][2] * j3[2][3]) + r_0_2[0][3]},
        {(r_0_2[1][0] * j3[0][0]) + (r_0_2[1][1] * j3[1][0]) + (r_0_2[1][2] * j3[2][0]), (r_0_2[1][0] * j3[0][1]) + (r_0_2[1][1] * j3[1][1]) + (r_0_2[1][2] * j3[2][1]), (r_0_2[1][0] * j3[0][2]) + (r_0_2[1][1] * j3[1][2]) + (r_0_2[1][2] * j3[2][2]), (r_0_2[1][0] * j3[0][3]) + (r_0_2[1][1] * j3[1][3]) + (r_0_2[1][2] * j3[2][3]) + r_0_2[1][3]},
        {(r_0_2[2][0] * j3[0][0]) + (r_0_2[2][1] * j3[1][0]) + (r_0_2[2][2] * j3[2][0]), (r_0_2[2][0] * j3[0][1]) + (r_0_2[2][1] * j3[1][1]) + (r_0_2[2][2] * j3[2][1]), (r_0_2[2][0] * j3[0][2]) + (r_0_2[2][1] * j3[1][2]) + (r_0_2[2][2] * j3[2][2]), (r_0_2[2][0] * j3[0][3]) + (r_0_2[2][1] * j3[1][3]) + (r_0_2[2][2] * j3[2][3]) + r_0_2[2][3]},
    }; //{0, 0, 0, 1}};

    const double r_3_6[3][3] = {
        {(r_0_3[0][0] * r_0_5[0][0]) + (r_0_3[1][0] * r_0_5[1][0]) + (r_0_3[2][0] * r_0_5[2][0]), (r_0_3[0][0] * r_0_5[0][1]) + (r_0_3[1][0] * r_0_5[1][1]) + (r_0_3[2][0] * r_0_5[2][1]), (r_0_3[0][0] * r_0_5[0][2]) + (r_0_3[1][0] * r_0_5[1][2]) + (r_0_3[2][0] * r_0_5[2][2])},
        {(r_0_3[0][1] * r_0_5[0][0]) + (r_0_3[1][1] * r_0_5[1][0]) + (r_0_3[2][1] * r_0_5[2][0]), (r_0_3[0][1] * r_0_5[0][1]) + (r_0_3[1][1] * r_0_5[1][1]) + (r_0_3[2][1] * r_0_5[2][1]), (r_0_3[0][1] * r_0_5[0][2]) + (r_0_3[1][1] * r_0_5[1][2]) + (r_0_3[2][1] * r_0_5[2][2])},
        {(r_0_3[0][2] * r_0_5[0][0]) + (r_0_3[1][2] * r_0_5[1][0]) + (r_0_3[2][2] * r_0_5[2][0]), (r_0_3[0][2] * r_0_5[0][1]) + (r_0_3[1][2] * r_0_5[1][1]) + (r_0_3[2][2] * r_0_5[2][1]), (r_0_3[0][2] * r_0_5[0][2]) + (r_0_3[1][2] * r_0_5[1][2]) + (r_0_3[2][2] * r_0_5[2][2])}};

    // J5
    const double R8 = degrees(atan2(r_3_6[1][2], +sqrt(1 - sq(r_3_6[1][2]))));
    const double S8 = degrees(atan2(r_3_6[1][2], -sqrt(1 - sq(r_3_6[1][2]))));
    solution[4] = ((S8 /*B8*/ > 0 && R8 > 0) ? R8 : S8); //TYPO? https://github.com/Chris-Annin/AR2/issues/23

    // J4
    solution[3] = (solution[4] > 0 ? degrees(atan2(r_3_6[0][2], r_3_6[1][2])) : degrees(atan2(-r_3_6[0][2], -r_3_6[1][2])));

    // J6
    const double R9 = (r_3_6[2][1] < 0 ? degrees(atan2(r_3_6[2][0], -r_3_6[2][1])) - 180 : degrees(atan2(r_3_6[2][0], -r_3_6[2][1])) + 180);
    const double S9 = (r_3_6[2][1] < 0 ? degrees(atan2(-r_3_6[2][0], r_3_6[2][1])) + 180 : degrees(atan2(-r_3_6[2][0], r_3_6[2][1])) - 180);
    solution[5] = (solution[4] < 0 ? S9 : R9);
}

void forward(frame_t *solution, double *jointAngles)
{
    double j1[4][4] = {{cos(jointAngles[0]), -sin(jointAngles[0]) * cos_alpha[0], sin(jointAngles[0]) * sin_alpha[0], dh_params[0].a * cos(jointAngles[0])},
                       {sin(jointAngles[0]), cos(jointAngles[0]) * cos_alpha[0], -cos(jointAngles[0]) * sin_alpha[0], dh_params[0].a * sin(jointAngles[0])},
                       {0, sin_alpha[0], cos_alpha[0], dh_params[0].d},
                       {0, 0, 0, 1}};
    double j2[4][4] = {{cos(jointAngles[1]), -sin(jointAngles[1]) * cos_alpha[1], sin(jointAngles[1]) * sin_alpha[1], dh_params[1].a * cos(jointAngles[1])},
                       {sin(jointAngles[1]), cos(jointAngles[1]) * cos_alpha[1], -cos(jointAngles[1]) * sin_alpha[1], dh_params[1].a * sin(jointAngles[1])},
                       {0, sin_alpha[1], cos_alpha[1], dh_params[1].d},
                       {0, 0, 0, 1}};
    double j3[4][4] = {{cos(jointAngles[2]), -sin(jointAngles[2]) * cos_alpha[2], sin(jointAngles[2]) * sin_alpha[2], dh_params[2].a * cos(jointAngles[2])},
                       {sin(jointAngles[2]), cos(jointAngles[2]) * cos_alpha[2], -cos(jointAngles[2]) * sin_alpha[2], dh_params[2].a * sin(jointAngles[2])},
                       {0, sin_alpha[2], cos_alpha[2], dh_params[2].d},
                       {0, 0, 0, 1}};
    double j4[4][4] = {{cos(jointAngles[3]), -sin(jointAngles[3]) * cos_alpha[3], sin(jointAngles[3]) * sin_alpha[3], dh_params[3].a * cos(jointAngles[3])},
                       {sin(jointAngles[3]), cos(jointAngles[3]) * cos_alpha[3], -cos(jointAngles[3]) * sin_alpha[3], dh_params[3].a * sin(jointAngles[3])},
                       {0, sin_alpha[3], cos_alpha[3], dh_params[3].d},
                       {0, 0, 0, 1}};
    double j5[4][4] = {{cos(jointAngles[4]), -sin(jointAngles[4]) * cos_alpha[4], sin(jointAngles[4]) * sin_alpha[4], dh_params[4].a * cos(jointAngles[4])},
                       {sin(jointAngles[4]), cos(jointAngles[4]) * cos_alpha[4], -cos(jointAngles[4]) * sin_alpha[4], dh_params[4].a * sin(jointAngles[4])},
                       {0, sin_alpha[4], cos_alpha[4], dh_params[4].d},
                       {0, 0, 0, 1}};
    double j6[4][4] = {{cos(jointAngles[5]), -sin(jointAngles[5]) * cos_alpha[5], sin(jointAngles[5]) * sin_alpha[5], dh_params[5].a * cos(jointAngles[5])},
                       {sin(jointAngles[5]), cos(jointAngles[5]) * cos_alpha[5], -cos(jointAngles[5]) * sin_alpha[5], dh_params[5].a * sin(jointAngles[5])},
                       {0, sin_alpha[5], cos_alpha[5], dh_params[5].d},
                       {0, 0, 0, 1}};

    double r_0_1[4][4] = {{(_wf[0][0] * j1[0][0]) + (_wf[0][1] * j1[1][0]) + (_wf[0][2] * j1[2][0]) + (_wf[0][3] * j1[3][0]), (_wf[0][0] * j1[0][1]) + (_wf[0][1] * j1[1][1]) + (_wf[0][2] * j1[2][1]) + (_wf[0][3] * j1[3][1]), (_wf[0][0] * j1[0][2]) + (_wf[0][1] * j1[1][2]) + (_wf[0][2] * j1[2][2]) + (_wf[0][3] * j1[3][2]), (_wf[0][0] * j1[0][3]) + (_wf[0][1] * j1[1][3]) + (_wf[0][2] * j1[2][3]) + (_wf[0][3] * j1[3][3])},
                          {(_wf[1][0] * j1[0][0]) + (_wf[1][1] * j1[1][0]) + (_wf[1][2] * j1[2][0]) + (_wf[1][3] * j1[3][0]), (_wf[1][0] * j1[0][1]) + (_wf[1][1] * j1[1][1]) + (_wf[1][2] * j1[2][1]) + (_wf[1][3] * j1[3][1]), (_wf[1][0] * j1[0][2]) + (_wf[1][1] * j1[1][2]) + (_wf[1][2] * j1[2][2]) + (_wf[1][3] * j1[3][2]), (_wf[1][0] * j1[0][3]) + (_wf[1][1] * j1[1][3]) + (_wf[1][2] * j1[2][3]) + (_wf[1][3] * j1[3][3])},
                          {(_wf[2][0] * j1[0][0]) + (_wf[2][1] * j1[1][0]) + (_wf[2][2] * j1[2][0]) + (_wf[2][3] * j1[3][0]), (_wf[2][0] * j1[0][1]) + (_wf[2][1] * j1[1][1]) + (_wf[2][2] * j1[2][1]) + (_wf[2][3] * j1[3][1]), (_wf[2][0] * j1[0][2]) + (_wf[2][1] * j1[1][2]) + (_wf[2][2] * j1[2][2]) + (_wf[2][3] * j1[3][2]), (_wf[2][0] * j1[0][3]) + (_wf[2][1] * j1[1][3]) + (_wf[2][2] * j1[2][3]) + (_wf[2][3] * j1[3][3])},
                          {(_wf[3][0] * j1[0][0]) + (_wf[3][1] * j1[1][0]) + (_wf[3][2] * j1[2][0]) + (_wf[3][3] * j1[3][0]), (_wf[3][0] * j1[0][1]) + (_wf[3][1] * j1[1][1]) + (_wf[3][2] * j1[2][1]) + (_wf[3][3] * j1[3][1]), (_wf[3][0] * j1[0][2]) + (_wf[3][1] * j1[1][2]) + (_wf[3][2] * j1[2][2]) + (_wf[3][3] * j1[3][2]), (_wf[3][0] * j1[0][3]) + (_wf[3][1] * j1[1][3]) + (_wf[3][2] * j1[2][3]) + (_wf[3][3] * j1[3][3])}};

    double r_0_2[4][4] = {{(r_0_1[0][0] * j2[0][0]) + (r_0_1[0][1] * j2[1][0]) + (r_0_1[0][2] * j2[2][0]) + (r_0_1[0][3] * j2[3][0]), (r_0_1[0][0] * j2[0][1]) + (r_0_1[0][1] * j2[1][1]) + (r_0_1[0][2] * j2[2][1]) + (r_0_1[0][3] * j2[3][1]), (r_0_1[0][0] * j2[0][2]) + (r_0_1[0][1] * j2[1][2]) + (r_0_1[0][2] * j2[2][2]) + (r_0_1[0][3] * j2[3][2]), (r_0_1[0][0] * j2[0][3]) + (r_0_1[0][1] * j2[1][3]) + (r_0_1[0][2] * j2[2][3]) + (r_0_1[0][3] * j2[3][3])},
                          {(r_0_1[1][0] * j2[0][0]) + (r_0_1[1][1] * j2[1][0]) + (r_0_1[1][2] * j2[2][0]) + (r_0_1[1][3] * j2[3][0]), (r_0_1[1][0] * j2[0][1]) + (r_0_1[1][1] * j2[1][1]) + (r_0_1[1][2] * j2[2][1]) + (r_0_1[1][3] * j2[3][1]), (r_0_1[1][0] * j2[0][2]) + (r_0_1[1][1] * j2[1][2]) + (r_0_1[1][2] * j2[2][2]) + (r_0_1[1][3] * j2[3][2]), (r_0_1[1][0] * j2[0][3]) + (r_0_1[1][1] * j2[1][3]) + (r_0_1[1][2] * j2[2][3]) + (r_0_1[1][3] * j2[3][3])},
                          {(r_0_1[2][0] * j2[0][0]) + (r_0_1[2][1] * j2[1][0]) + (r_0_1[2][2] * j2[2][0]) + (r_0_1[2][3] * j2[3][0]), (r_0_1[2][0] * j2[0][1]) + (r_0_1[2][1] * j2[1][1]) + (r_0_1[2][2] * j2[2][1]) + (r_0_1[2][3] * j2[3][1]), (r_0_1[2][0] * j2[0][2]) + (r_0_1[2][1] * j2[1][2]) + (r_0_1[2][2] * j2[2][2]) + (r_0_1[2][3] * j2[3][2]), (r_0_1[2][0] * j2[0][3]) + (r_0_1[2][1] * j2[1][3]) + (r_0_1[2][2] * j2[2][3]) + (r_0_1[2][3] * j2[3][3])},
                          {(r_0_1[3][0] * j2[0][0]) + (r_0_1[3][1] * j2[1][0]) + (r_0_1[3][2] * j2[2][0]) + (r_0_1[3][3] * j2[3][0]), (r_0_1[3][0] * j2[0][1]) + (r_0_1[3][1] * j2[1][1]) + (r_0_1[3][2] * j2[2][1]) + (r_0_1[3][3] * j2[3][1]), (r_0_1[3][0] * j2[0][2]) + (r_0_1[3][1] * j2[1][2]) + (r_0_1[3][2] * j2[2][2]) + (r_0_1[3][3] * j2[3][2]), (r_0_1[3][0] * j2[0][3]) + (r_0_1[3][1] * j2[1][3]) + (r_0_1[3][2] * j2[2][3]) + (r_0_1[3][3] * j2[3][3])}};

    double r_0_3[4][4] = {{(r_0_2[0][0] * j3[0][0]) + (r_0_2[0][1] * j3[1][0]) + (r_0_2[0][2] * j3[2][0]) + (r_0_2[0][3] * j3[3][0]), (r_0_2[0][0] * j3[0][1]) + (r_0_2[0][1] * j3[1][1]) + (r_0_2[0][2] * j3[2][1]) + (r_0_2[0][3] * j3[3][1]), (r_0_2[0][0] * j3[0][2]) + (r_0_2[0][1] * j3[1][2]) + (r_0_2[0][2] * j3[2][2]) + (r_0_2[0][3] * j3[3][2]), (r_0_2[0][0] * j3[0][3]) + (r_0_2[0][1] * j3[1][3]) + (r_0_2[0][2] * j3[2][3]) + (r_0_2[0][3] * j3[3][3])},
                          {(r_0_2[1][0] * j3[0][0]) + (r_0_2[1][1] * j3[1][0]) + (r_0_2[1][2] * j3[2][0]) + (r_0_2[1][3] * j3[3][0]), (r_0_2[1][0] * j3[0][1]) + (r_0_2[1][1] * j3[1][1]) + (r_0_2[1][2] * j3[2][1]) + (r_0_2[1][3] * j3[3][1]), (r_0_2[1][0] * j3[0][2]) + (r_0_2[1][1] * j3[1][2]) + (r_0_2[1][2] * j3[2][2]) + (r_0_2[1][3] * j3[3][2]), (r_0_2[1][0] * j3[0][3]) + (r_0_2[1][1] * j3[1][3]) + (r_0_2[1][2] * j3[2][3]) + (r_0_2[1][3] * j3[3][3])},
                          {(r_0_2[2][0] * j3[0][0]) + (r_0_2[2][1] * j3[1][0]) + (r_0_2[2][2] * j3[2][0]) + (r_0_2[2][3] * j3[3][0]), (r_0_2[2][0] * j3[0][1]) + (r_0_2[2][1] * j3[1][1]) + (r_0_2[2][2] * j3[2][1]) + (r_0_2[2][3] * j3[3][1]), (r_0_2[2][0] * j3[0][2]) + (r_0_2[2][1] * j3[1][2]) + (r_0_2[2][2] * j3[2][2]) + (r_0_2[2][3] * j3[3][2]), (r_0_2[2][0] * j3[0][3]) + (r_0_2[2][1] * j3[1][3]) + (r_0_2[2][2] * j3[2][3]) + (r_0_2[2][3] * j3[3][3])},
                          {(r_0_2[3][0] * j3[0][0]) + (r_0_2[3][1] * j3[1][0]) + (r_0_2[3][2] * j3[2][0]) + (r_0_2[3][3] * j3[3][0]), (r_0_2[3][0] * j3[0][1]) + (r_0_2[3][1] * j3[1][1]) + (r_0_2[3][2] * j3[2][1]) + (r_0_2[3][3] * j3[3][1]), (r_0_2[3][0] * j3[0][2]) + (r_0_2[3][1] * j3[1][2]) + (r_0_2[3][2] * j3[2][2]) + (r_0_2[3][3] * j3[3][2]), (r_0_2[3][0] * j3[0][3]) + (r_0_2[3][1] * j3[1][3]) + (r_0_2[3][2] * j3[2][3]) + (r_0_2[3][3] * j3[3][3])}};

    double r_0_4[4][4] = {{(r_0_3[0][0] * j4[0][0]) + (r_0_3[0][1] * j4[1][0]) + (r_0_3[0][2] * j4[2][0]) + (r_0_3[0][3] * j4[3][0]), (r_0_3[0][0] * j4[0][1]) + (r_0_3[0][1] * j4[1][1]) + (r_0_3[0][2] * j4[2][1]) + (r_0_3[0][3] * j4[3][1]), (r_0_3[0][0] * j4[0][2]) + (r_0_3[0][1] * j4[1][2]) + (r_0_3[0][2] * j4[2][2]) + (r_0_3[0][3] * j4[3][2]), (r_0_3[0][0] * j4[0][3]) + (r_0_3[0][1] * j4[1][3]) + (r_0_3[0][2] * j4[2][3]) + (r_0_3[0][3] * j4[3][3])},
                          {(r_0_3[1][0] * j4[0][0]) + (r_0_3[1][1] * j4[1][0]) + (r_0_3[1][2] * j4[2][0]) + (r_0_3[1][3] * j4[3][0]), (r_0_3[1][0] * j4[0][1]) + (r_0_3[1][1] * j4[1][1]) + (r_0_3[1][2] * j4[2][1]) + (r_0_3[1][3] * j4[3][1]), (r_0_3[1][0] * j4[0][2]) + (r_0_3[1][1] * j4[1][2]) + (r_0_3[1][2] * j4[2][2]) + (r_0_3[1][3] * j4[3][2]), (r_0_3[1][0] * j4[0][3]) + (r_0_3[1][1] * j4[1][3]) + (r_0_3[1][2] * j4[2][3]) + (r_0_3[1][3] * j4[3][3])},
                          {(r_0_3[2][0] * j4[0][0]) + (r_0_3[2][1] * j4[1][0]) + (r_0_3[2][2] * j4[2][0]) + (r_0_3[2][3] * j4[3][0]), (r_0_3[2][0] * j4[0][1]) + (r_0_3[2][1] * j4[1][1]) + (r_0_3[2][2] * j4[2][1]) + (r_0_3[2][3] * j4[3][1]), (r_0_3[2][0] * j4[0][2]) + (r_0_3[2][1] * j4[1][2]) + (r_0_3[2][2] * j4[2][2]) + (r_0_3[2][3] * j4[3][2]), (r_0_3[2][0] * j4[0][3]) + (r_0_3[2][1] * j4[1][3]) + (r_0_3[2][2] * j4[2][3]) + (r_0_3[2][3] * j4[3][3])},
                          {(r_0_3[3][0] * j4[0][0]) + (r_0_3[3][1] * j4[1][0]) + (r_0_3[3][2] * j4[2][0]) + (r_0_3[3][3] * j4[3][0]), (r_0_3[3][0] * j4[0][1]) + (r_0_3[3][1] * j4[1][1]) + (r_0_3[3][2] * j4[2][1]) + (r_0_3[3][3] * j4[3][1]), (r_0_3[3][0] * j4[0][2]) + (r_0_3[3][1] * j4[1][2]) + (r_0_3[3][2] * j4[2][2]) + (r_0_3[3][3] * j4[3][2]), (r_0_3[3][0] * j4[0][3]) + (r_0_3[3][1] * j4[1][3]) + (r_0_3[3][2] * j4[2][3]) + (r_0_3[3][3] * j4[3][3])}};

    double r_0_5[4][4] = {{(r_0_4[0][0] * j5[0][0]) + (r_0_4[0][1] * j5[1][0]) + (r_0_4[0][2] * j5[2][0]) + (r_0_4[0][3] * j5[3][0]), (r_0_4[0][0] * j5[0][1]) + (r_0_4[0][1] * j5[1][1]) + (r_0_4[0][2] * j5[2][1]) + (r_0_4[0][3] * j5[3][1]), (r_0_4[0][0] * j5[0][2]) + (r_0_4[0][1] * j5[1][2]) + (r_0_4[0][2] * j5[2][2]) + (r_0_4[0][3] * j5[3][2]), (r_0_4[0][0] * j5[0][3]) + (r_0_4[0][1] * j5[1][3]) + (r_0_4[0][2] * j5[2][3]) + (r_0_4[0][3] * j5[3][3])},
                          {(r_0_4[1][0] * j5[0][0]) + (r_0_4[1][1] * j5[1][0]) + (r_0_4[1][2] * j5[2][0]) + (r_0_4[1][3] * j5[3][0]), (r_0_4[1][0] * j5[0][1]) + (r_0_4[1][1] * j5[1][1]) + (r_0_4[1][2] * j5[2][1]) + (r_0_4[1][3] * j5[3][1]), (r_0_4[1][0] * j5[0][2]) + (r_0_4[1][1] * j5[1][2]) + (r_0_4[1][2] * j5[2][2]) + (r_0_4[1][3] * j5[3][2]), (r_0_4[1][0] * j5[0][3]) + (r_0_4[1][1] * j5[1][3]) + (r_0_4[1][2] * j5[2][3]) + (r_0_4[1][3] * j5[3][3])},
                          {(r_0_4[2][0] * j5[0][0]) + (r_0_4[2][1] * j5[1][0]) + (r_0_4[2][2] * j5[2][0]) + (r_0_4[2][3] * j5[3][0]), (r_0_4[2][0] * j5[0][1]) + (r_0_4[2][1] * j5[1][1]) + (r_0_4[2][2] * j5[2][1]) + (r_0_4[2][3] * j5[3][1]), (r_0_4[2][0] * j5[0][2]) + (r_0_4[2][1] * j5[1][2]) + (r_0_4[2][2] * j5[2][2]) + (r_0_4[2][3] * j5[3][2]), (r_0_4[2][0] * j5[0][3]) + (r_0_4[2][1] * j5[1][3]) + (r_0_4[2][2] * j5[2][3]) + (r_0_4[2][3] * j5[3][3])},
                          {(r_0_4[3][0] * j5[0][0]) + (r_0_4[3][1] * j5[1][0]) + (r_0_4[3][2] * j5[2][0]) + (r_0_4[3][3] * j5[3][0]), (r_0_4[3][0] * j5[0][1]) + (r_0_4[3][1] * j5[1][1]) + (r_0_4[3][2] * j5[2][1]) + (r_0_4[3][3] * j5[3][1]), (r_0_4[3][0] * j5[0][2]) + (r_0_4[3][1] * j5[1][2]) + (r_0_4[3][2] * j5[2][2]) + (r_0_4[3][3] * j5[3][2]), (r_0_4[3][0] * j5[0][3]) + (r_0_4[3][1] * j5[1][3]) + (r_0_4[3][2] * j5[2][3]) + (r_0_4[3][3] * j5[3][3])}};

    double r_0_6[4][4] = {{(r_0_5[0][0] * j6[0][0]) + (r_0_5[0][1] * j6[1][0]) + (r_0_5[0][2] * j6[2][0]) + (r_0_5[0][3] * j6[3][0]), (r_0_5[0][0] * j6[0][1]) + (r_0_5[0][1] * j6[1][1]) + (r_0_5[0][2] * j6[2][1]) + (r_0_5[0][3] * j6[3][1]), (r_0_5[0][0] * j6[0][2]) + (r_0_5[0][1] * j6[1][2]) + (r_0_5[0][2] * j6[2][2]) + (r_0_5[0][3] * j6[3][2]), (r_0_5[0][0] * j6[0][3]) + (r_0_5[0][1] * j6[1][3]) + (r_0_5[0][2] * j6[2][3]) + (r_0_5[0][3] * j6[3][3])},
                          {(r_0_5[1][0] * j6[0][0]) + (r_0_5[1][1] * j6[1][0]) + (r_0_5[1][2] * j6[2][0]) + (r_0_5[1][3] * j6[3][0]), (r_0_5[1][0] * j6[0][1]) + (r_0_5[1][1] * j6[1][1]) + (r_0_5[1][2] * j6[2][1]) + (r_0_5[1][3] * j6[3][1]), (r_0_5[1][0] * j6[0][2]) + (r_0_5[1][1] * j6[1][2]) + (r_0_5[1][2] * j6[2][2]) + (r_0_5[1][3] * j6[3][2]), (r_0_5[1][0] * j6[0][3]) + (r_0_5[1][1] * j6[1][3]) + (r_0_5[1][2] * j6[2][3]) + (r_0_5[1][3] * j6[3][3])},
                          {(r_0_5[2][0] * j6[0][0]) + (r_0_5[2][1] * j6[1][0]) + (r_0_5[2][2] * j6[2][0]) + (r_0_5[2][3] * j6[3][0]), (r_0_5[2][0] * j6[0][1]) + (r_0_5[2][1] * j6[1][1]) + (r_0_5[2][2] * j6[2][1]) + (r_0_5[2][3] * j6[3][1]), (r_0_5[2][0] * j6[0][2]) + (r_0_5[2][1] * j6[1][2]) + (r_0_5[2][2] * j6[2][2]) + (r_0_5[2][3] * j6[3][2]), (r_0_5[2][0] * j6[0][3]) + (r_0_5[2][1] * j6[1][3]) + (r_0_5[2][2] * j6[2][3]) + (r_0_5[2][3] * j6[3][3])},
                          {(r_0_5[3][0] * j6[0][0]) + (r_0_5[3][1] * j6[1][0]) + (r_0_5[3][2] * j6[2][0]) + (r_0_5[3][3] * j6[3][0]), (r_0_5[3][0] * j6[0][1]) + (r_0_5[3][1] * j6[1][1]) + (r_0_5[3][2] * j6[2][1]) + (r_0_5[3][3] * j6[3][1]), (r_0_5[3][0] * j6[0][2]) + (r_0_5[3][1] * j6[1][2]) + (r_0_5[3][2] * j6[2][2]) + (r_0_5[3][3] * j6[3][2]), (r_0_5[3][0] * j6[0][3]) + (r_0_5[3][1] * j6[1][3]) + (r_0_5[3][2] * j6[2][3]) + (r_0_5[3][3] * j6[3][3])}};

    double r_0_t[4][4] = {{(r_0_6[0][0] * _tf[0][0]) + (r_0_6[0][1] * _tf[1][0]) + (r_0_6[0][2] * _tf[2][0]) + (r_0_6[0][3] * _tf[3][0]), (r_0_6[0][0] * _tf[0][1]) + (r_0_6[0][1] * _tf[1][1]) + (r_0_6[0][2] * _tf[2][1]) + (r_0_6[0][3] * _tf[3][1]), (r_0_6[0][0] * _tf[0][2]) + (r_0_6[0][1] * _tf[1][2]) + (r_0_6[0][2] * _tf[2][2]) + (r_0_6[0][3] * _tf[3][2]), (r_0_6[0][0] * _tf[0][3]) + (r_0_6[0][1] * _tf[1][3]) + (r_0_6[0][2] * _tf[2][3]) + (r_0_6[0][3] * _tf[3][3])},
                          {(r_0_6[1][0] * _tf[0][0]) + (r_0_6[1][1] * _tf[1][0]) + (r_0_6[1][2] * _tf[2][0]) + (r_0_6[1][3] * _tf[3][0]), (r_0_6[1][0] * _tf[0][1]) + (r_0_6[1][1] * _tf[1][1]) + (r_0_6[1][2] * _tf[2][1]) + (r_0_6[1][3] * _tf[3][1]), (r_0_6[1][0] * _tf[0][2]) + (r_0_6[1][1] * _tf[1][2]) + (r_0_6[1][2] * _tf[2][2]) + (r_0_6[1][3] * _tf[3][2]), (r_0_6[1][0] * _tf[0][3]) + (r_0_6[1][1] * _tf[1][3]) + (r_0_6[1][2] * _tf[2][3]) + (r_0_6[1][3] * _tf[3][3])},
                          {(r_0_6[2][0] * _tf[0][0]) + (r_0_6[2][1] * _tf[1][0]) + (r_0_6[2][2] * _tf[2][0]) + (r_0_6[2][3] * _tf[3][0]), (r_0_6[2][0] * _tf[0][1]) + (r_0_6[2][1] * _tf[1][1]) + (r_0_6[2][2] * _tf[2][1]) + (r_0_6[2][3] * _tf[3][1]), (r_0_6[2][0] * _tf[0][2]) + (r_0_6[2][1] * _tf[1][2]) + (r_0_6[2][2] * _tf[2][2]) + (r_0_6[2][3] * _tf[3][2]), (r_0_6[2][0] * _tf[0][3]) + (r_0_6[2][1] * _tf[1][3]) + (r_0_6[2][2] * _tf[2][3]) + (r_0_6[2][3] * _tf[3][3])},
                          {(r_0_6[3][0] * _tf[0][0]) + (r_0_6[3][1] * _tf[1][0]) + (r_0_6[3][2] * _tf[2][0]) + (r_0_6[3][3] * _tf[3][0]), (r_0_6[3][0] * _tf[0][1]) + (r_0_6[3][1] * _tf[1][1]) + (r_0_6[3][2] * _tf[2][1]) + (r_0_6[3][3] * _tf[3][1]), (r_0_6[3][0] * _tf[0][2]) + (r_0_6[3][1] * _tf[1][2]) + (r_0_6[3][2] * _tf[2][2]) + (r_0_6[3][3] * _tf[3][2]), (r_0_6[3][0] * _tf[0][3]) + (r_0_6[3][1] * _tf[1][3]) + (r_0_6[3][2] * _tf[2][3]) + (r_0_6[3][3] * _tf[3][3])}};

    solution->x = r_0_t[0][3];
    solution->y = r_0_t[1][3];
    solution->z = r_0_t[2][3];

    double I8 = degrees(atan2(-r_0_t[3][1], sqrt(sq(r_0_t[0][2]) + sq(r_0_t[1][2]))));

    solution->yaw = degrees(atan2((r_0_t[2][1] / I8), (r_0_t[1][1] / I8)));
    solution->pitch = I8;
    solution->roll = degrees(atan2((r_0_t[1][2] / I8), (r_0_t[0][3] / I8)));
}
