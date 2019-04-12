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
    {0.0, PI / 2, 140.25, 90.0},
    {0.0, 0.0, 0.0, 250.0},
    {0.0, PI / 2, 0.0, 66.125},
    {0.0, PI / 2, 342.451, 0.0},
    {0.0, PI / 2, 0.0, 0.0},
    {0.0, PI, 0.0, 61.350}};

const ik_config_t sin_params[] = {
    {sin(dh_params[0].theta), sin(dh_params[0].alpha), sin(dh_params[0].d), sin(dh_params[0].a)},
    {sin(dh_params[1].theta), sin(dh_params[1].alpha), sin(dh_params[1].d), sin(dh_params[1].a)},
    {sin(dh_params[2].theta), sin(dh_params[2].alpha), sin(dh_params[2].d), sin(dh_params[2].a)},
    {sin(dh_params[3].theta), sin(dh_params[3].alpha), sin(dh_params[3].d), sin(dh_params[3].a)},
    {sin(dh_params[4].theta), sin(dh_params[4].alpha), sin(dh_params[4].d), sin(dh_params[4].a)},
    {sin(dh_params[5].theta), sin(dh_params[5].alpha), sin(dh_params[5].d), sin(dh_params[5].a)}};

const ik_config_t cos_params[] = {
    {cos(dh_params[0].theta), cos(dh_params[0].alpha), cos(dh_params[0].d), cos(dh_params[0].a)},
    {cos(dh_params[1].theta), cos(dh_params[1].alpha), cos(dh_params[1].d), cos(dh_params[1].a)},
    {cos(dh_params[2].theta), cos(dh_params[2].alpha), cos(dh_params[2].d), cos(dh_params[2].a)},
    {cos(dh_params[3].theta), cos(dh_params[3].alpha), cos(dh_params[3].d), cos(dh_params[3].a)},
    {cos(dh_params[4].theta), cos(dh_params[4].alpha), cos(dh_params[4].d), cos(dh_params[4].a)},
    {cos(dh_params[5].theta), cos(dh_params[5].alpha), cos(dh_params[5].d), cos(dh_params[5].a)}};

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

/** 
 * Translate / rotate the tool origin to the given location in the world coordinate frame. 
 * Yaw, Pitch and Roll are given in degrees.
 * 
 * Solution is returned as an array of doubles, representing the rotation (in degrees) 
 * of each joint in the IK chain.
 **/
void inverse(double *solution, double x_pos, double y_pos, double z_pos, double roll, double pitch, double yaw)
{
    static const double _r_r_0_6[3][4] = {
        {-1, sin(radians(180)), 0, 0},
        {-sin(radians(180)) * cos_params[5].alpha, -cos_params[5].alpha, sin_params[5].alpha, 0},
        {sin(radians(180)) * sin_params[5].alpha, sin_params[5].alpha, cos_params[5].alpha, -dh_params[5].d},
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
        {cos(radians(j1deg)), -sin(radians(j1deg)) * cos_params[0].alpha, sin(radians(j1deg)) * sin_params[0].alpha, dh_params[0].a * cos(radians(j1deg))},
        {sin(radians(j1deg)), cos(radians(j1deg)) * cos_params[0].alpha, -cos(radians(j1deg)) * sin_params[0].alpha, dh_params[0].a * sin(radians(j1deg))},
        {0, sin_params[0].alpha, cos_params[0].alpha, dh_params[0].d},
    }; //{0, 0, 0, 1}};

    const double r_0_1[3][4] = {
        {(_wf[0][0] * j1[0][0]) + (_wf[0][1] * j1[1][0]), (_wf[0][0] * j1[0][1]) + (_wf[0][1] * j1[1][1]) + (_wf[0][2] * j1[2][1]), (_wf[0][0] * j1[0][2]) + (_wf[0][1] * j1[1][2]) + (_wf[0][2] * j1[2][2]), (_wf[0][0] * j1[0][3]) + (_wf[0][1] * j1[1][3]) + (_wf[0][2] * j1[2][3]) + _wf[0][3]},
        {(_wf[1][0] * j1[0][0]) + (_wf[1][1] * j1[1][0]), (_wf[1][0] * j1[0][1]) + (_wf[1][1] * j1[1][1]) + (_wf[1][2] * j1[2][1]), (_wf[1][0] * j1[0][2]) + (_wf[1][1] * j1[1][2]) + (_wf[1][2] * j1[2][2]), (_wf[1][0] * j1[0][3]) + (_wf[1][1] * j1[1][3]) + (_wf[1][2] * j1[2][3]) + _wf[1][3]},
        {(_wf[2][0] * j1[0][0]) + (_wf[2][1] * j1[1][0]), (_wf[2][0] * j1[0][1]) + (_wf[2][1] * j1[1][1]) + (_wf[2][2] * j1[2][1]), (_wf[2][0] * j1[0][2]) + (_wf[2][1] * j1[1][2]) + (_wf[2][2] * j1[2][2]), (_wf[2][0] * j1[0][3]) + (_wf[2][1] * j1[1][3]) + (_wf[2][2] * j1[2][3]) + _wf[2][3]},
    }; //{0, 0, 0, 1}};

    // J2
    solution[1] = J2_theta[pX_a1[0] >= 0]; //(pX_a1[0] < 0 ? J2_theta[0] : J2_theta[1]);

    const double j2[3][4] = {
        {cos(radians(solution[1])), -sin(radians(solution[1])) * cos_params[1].alpha, sin(radians(solution[1])) * sin_params[1].alpha, dh_params[1].a * cos(radians(solution[1]))},
        {sin(radians(solution[1])), cos(radians(solution[1])) * cos_params[1].alpha, -cos(radians(solution[1])) * sin_params[1].alpha, dh_params[1].a * sin(radians(solution[1]))},
        {0, sin_params[1].alpha, cos_params[1].alpha, dh_params[1].d},
    }; //{0, 0, 0, 1}};

    // J3
    solution[2] = theta_C[pX_a1[0] >= 0]; //(pX_a1[0] < 0 ? theta_C[0] : theta_C[1]);
    const double j3[3][4] = {
        {cos(radians((solution[2]) - 90)), -sin(radians((solution[2]) - 90)) * cos_params[2].alpha, sin(radians((solution[2]) - 90)) * sin_params[2].alpha, dh_params[2].a * cos(radians((solution[2]) - 90))},
        {sin(radians((solution[2]) - 90)), cos(radians((solution[2]) - 90)) * cos_params[2].alpha, -cos(radians((solution[2]) - 90)) * sin_params[2].alpha, dh_params[2].a * sin(radians((solution[2]) - 90))},
        {0, sin_params[2].alpha, cos_params[2].alpha, dh_params[2].d},
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
    const double R8 = degrees(atan2(r_3_6[3][3], +sqrt(1 - sq(r_3_6[3][3]))));
    const double S8 = degrees(atan2(r_3_6[3][3], -sqrt(1 - sq(r_3_6[3][3]))));
    solution[4] = ((S8 /*B8*/ > 0 && R8 > 0) ? R8 : S8); //TYPO? https://github.com/Chris-Annin/AR2/issues/23

    // J4
    solution[3] = (solution[4] > 0 ? degrees(atan2(r_3_6[0][2], r_3_6[1][2])) : degrees(atan2(-r_3_6[0][2], -r_3_6[1][2])));

    // J6
    const double R9 = (r_3_6[2][1] < 0 ? degrees(atan2(r_3_6[2][0], -r_3_6[2][1])) - 180 : degrees(atan2(r_3_6[2][0], -r_3_6[2][1])) + 180);
    const double S9 = (r_3_6[2][1] < 0 ? degrees(atan2(-r_3_6[2][0], r_3_6[2][1])) + 180 : degrees(atan2(-r_3_6[2][0], r_3_6[2][1])) - 180);
    solution[5] = (solution[4] < 0 ? S9 : R9);
}

/**
 * Given a set of joint angles, return the end effector's positionx/y/z/yaw/pitch/roll in the world frame.
 **/
void forward(frame_t *solution, double *jointAngles)
{
    double j1[4][4] = {{cos_params[0].theta, -sin_params[0].theta * cos_params[0].alpha, sin_params[0].theta * sin_params[0].alpha, dh_params[0].a * cos_params[0].theta},
                       {sin_params[0].theta, cos_params[0].theta * cos_params[0].alpha, -cos_params[0].theta * sin_params[0].alpha, dh_params[0].a * sin_params[0].theta},
                       {0, sin_params[0].alpha, cos_params[0].alpha, dh_params[0].d},
                       {0, 0, 0, 1}};
    double j2[4][4] = {{cos_params[1].theta, -sin_params[1].theta * cos_params[1].alpha, sin_params[1].theta * sin_params[1].alpha, dh_params[1].a * cos_params[1].theta},
                       {sin_params[1].theta, cos_params[1].theta * cos_params[1].alpha, -cos_params[1].theta * sin_params[1].alpha, dh_params[1].a * sin_params[1].theta},
                       {0, sin_params[1].alpha, cos_params[1].alpha, dh_params[1].d},
                       {0, 0, 0, 1}};
    double j3[4][4] = {{cos_params[2].theta, -sin_params[2].theta * cos_params[2].alpha, sin_params[2].theta * sin_params[2].alpha, dh_params[2].a * cos_params[2].theta},
                       {sin_params[2].theta, cos_params[2].theta * cos_params[2].alpha, -cos_params[2].theta * sin_params[2].alpha, dh_params[2].a * sin_params[2].theta},
                       {0, sin_params[2].alpha, cos_params[2].alpha, dh_params[2].d},
                       {0, 0, 0, 1}};
    double j4[4][4] = {{cos_params[3].theta, -sin_params[3].theta * cos_params[3].alpha, sin_params[3].theta * sin_params[3].alpha, dh_params[3].a * cos_params[3].theta},
                       {sin_params[3].theta, cos_params[3].theta * cos_params[3].alpha, -cos_params[3].theta * sin_params[3].alpha, dh_params[3].a * sin_params[3].theta},
                       {0, sin_params[3].alpha, cos_params[3].alpha, dh_params[3].d},
                       {0, 0, 0, 1}};
    double j5[4][4] = {{cos_params[4].theta, -sin_params[4].theta * cos_params[4].alpha, sin_params[4].theta * sin_params[4].alpha, dh_params[4].a * cos_params[4].theta},
                       {sin_params[4].theta, cos_params[4].theta * cos_params[4].alpha, -cos_params[4].theta * sin_params[4].alpha, dh_params[4].a * sin_params[4].theta},
                       {0, sin_params[4].alpha, cos_params[4].alpha, dh_params[4].d},
                       {0, 0, 0, 1}};
    double j6[4][4] = {{cos_params[5].theta, -sin_params[5].theta * cos_params[5].alpha, sin_params[5].theta * sin_params[5].alpha, dh_params[5].a * cos_params[5].theta},
                       {sin_params[5].theta, cos_params[5].theta * cos_params[5].alpha, -cos_params[5].theta * sin_params[5].alpha, dh_params[5].a * sin_params[5].theta},
                       {0, sin_params[5].alpha, cos_params[5].alpha, dh_params[5].d},
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

    double r_0_5[4][4] = {{(r_0_4[0][0] * B51) + (r_0_4[0][1] * B52) + (r_0_4[0][2] * B53) + (r_0_4[0][3] * B54), (r_0_4[0][0] * C51) + (r_0_4[0][1] * C52) + (r_0_4[0][2] * C53) + (r_0_4[0][3] * C54), (r_0_4[0][0] * D51) + (r_0_4[0][1] * D52) + (r_0_4[0][2] * D53) + (r_0_4[0][3] * D54), (r_0_4[0][0] * E51) + (r_0_4[0][1] * E52) + (r_0_4[0][2] * E53) + (r_0_4[0][3] * E54)},
                          {(r_0_4[1][0] * B51) + (r_0_4[1][1] * B52) + (r_0_4[1][2] * B53) + (r_0_4[1][3] * B54), (r_0_4[1][0] * C51) + (r_0_4[1][1] * C52) + (r_0_4[1][2] * C53) + (r_0_4[1][3] * C54), (r_0_4[1][0] * D51) + (r_0_4[1][1] * D52) + (r_0_4[1][2] * D53) + (r_0_4[1][3] * D54), (r_0_4[1][0] * E51) + (r_0_4[1][1] * E52) + (r_0_4[1][2] * E53) + (r_0_4[1][3] * E54)},
                          {(r_0_4[2][0] * B51) + (r_0_4[2][1] * B52) + (r_0_4[2][2] * B53) + (r_0_4[2][3] * B54), (r_0_4[2][0] * C51) + (r_0_4[2][1] * C52) + (r_0_4[2][2] * C53) + (r_0_4[2][3] * C54), (r_0_4[2][0] * D51) + (r_0_4[2][1] * D52) + (r_0_4[2][2] * D53) + (r_0_4[2][3] * D54), (r_0_4[2][0] * E51) + (r_0_4[2][1] * E52) + (r_0_4[2][2] * E53) + (r_0_4[2][3] * E54)},
                          {(r_0_4[3][0] * B51) + (r_0_4[3][1] * B52) + (r_0_4[3][2] * B53) + (r_0_4[3][3] * B54), (r_0_4[3][0] * C51) + (r_0_4[3][1] * C52) + (r_0_4[3][2] * C53) + (r_0_4[3][3] * C54), (r_0_4[3][0] * D51) + (r_0_4[3][1] * D52) + (r_0_4[3][2] * D53) + (r_0_4[3][3] * D54), (r_0_4[3][0] * E51) + (r_0_4[3][1] * E52) + (r_0_4[3][2] * E53) + (r_0_4[3][3] * E54)}};

    double r_0_6[4][4] = {{(G48 * B57) + (H48 * B58) + (I48 * B59) + (J48 * B60), (G48 * C57) + (H48 * C58) + (I48 * C59) + (J48 * C60), (G48 * D57) + (H48 * D58) + (I48 * D59) + (J48 * D60), (G48 * E57) + (H48 * E58) + (I48 * E59) + (J48 * E60)},
                          {(G49 * B57) + (H49 * B58) + (I49 * B59) + (J49 * B60), (G49 * C57) + (H49 * C58) + (I49 * C59) + (J49 * C60), (G49 * D57) + (H49 * D58) + (I49 * D59) + (J49 * D60), (G49 * E57) + (H49 * E58) + (I49 * E59) + (J49 * E60)},
                          {(G50 * B57) + (H50 * B58) + (I50 * B59) + (J50 * B60), (G50 * C57) + (H50 * C58) + (I50 * C59) + (J50 * C60), (G50 * D57) + (H50 * D58) + (I50 * D59) + (J50 * D60), (G50 * E57) + (H50 * E58) + (I50 * E59) + (J50 * E60)},
                          {(G51 * B57) + (H51 * B58) + (I51 * B59) + (J51 * B60), (G51 * C57) + (H51 * C58) + (I51 * C59) + (J51 * C60), (G51 * D57) + (H51 * D58) + (I51 * D59) + (J51 * D60), (G51 * E57) + (H51 * E58) + (I51 * E59) + (J51 * E60)}};

    double r_0_t[4][4] = {{(G54 * B63) + (H54 * B64) + (I54 * B65) + (J54 * B66), (G54 * C63) + (H54 * C64) + (I54 * C65) + (J54 * C66), (G54 * D63) + (H54 * D64) + (I54 * D65) + (J54 * D66), (G54 * E63) + (H54 * E64) + (I54 * E65) + (J54 * E66)},
                          {(G55 * B63) + (H55 * B64) + (I55 * B65) + (J55 * B66), (G55 * C63) + (H55 * C64) + (I55 * C65) + (J55 * C66), (G55 * D63) + (H55 * D64) + (I55 * D65) + (J55 * D66), (G55 * E63) + (H55 * E64) + (I55 * E65) + (J55 * E66)},
                          {(G56 * B63) + (H56 * B64) + (I56 * B65) + (J56 * B66), (G56 * C63) + (H56 * C64) + (I56 * C65) + (J56 * C66), (G56 * D63) + (H56 * D64) + (I56 * D65) + (J56 * D66), (G56 * E63) + (H56 * E64) + (I56 * E65) + (J56 * E66)},
                          {(G57 * B63) + (H57 * B64) + (I57 * B65) + (J57 * B66), (G57 * C63) + (H57 * C64) + (I57 * C65) + (J57 * C66), (G57 * D63) + (H57 * D64) + (I57 * D65) + (J57 * D66), (G57 * E63) + (H57 * E64) + (I57 * E65) + (J57 * E66)}};

    solution->x = r_0_t[0][3];
    solution->y = r_0_t[1][3];
    solution->z = r_0_t[2][3];

    double I8 = degrees(atan2(-r_0_t[3][1], sqrt(sq(r_0_t[0][2]) + sq(r_0_t[1][2]))));

    solution->yaw = degrees(atan2((r_0_t[2][1] / I8), (r_0_t[1][1] / I8)));
    solution->pitch = I8;
    solution->roll = degrees(atan2((r_0_t[1][2] / I8), (r_0_t[0][3] / I8)));
}

//Translate the tool origin (with no rotation) to the given location in the world coordinate frame
void translate(double *solution, double x_pos, double y_pos, double z_pos)
{
    // TODO
}

//Rotate the tool around the tool origin (with no translation).
void rotate(double *solution, double roll, double pitch, double yaw)
{
    // TODO
}