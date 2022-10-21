/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once
#include <iostream>

namespace apollo {
namespace drivers {
namespace robosense {

/**
 * @brief Order array for re-ordering point cloud.
 * Refer to suteng official manual
 */

const int REORDER_16[16] = {0, 8,  1, 9,  2, 10, 3, 11,
                            4, 12, 5, 13, 6, 14, 7, 15};

// suteng begin
const float SUTENG_VERT[16] = {-0.26208,  -0.227375,  -0.192101,  -0.157558,
                               -0.122632, -0.0880222, -0.0531994, -0.0176226,
                               0.261264,  0.226544,   0.192463,   0.157313,
                               0.12177,   0.087153,   0.0523267,  0.017373};

const float CHANNEL_NUM[16][41] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
// suteng calib_curves_str
const float INTENSITY_CAL[7][32] = {
    {15.43, 14.85, 13.03, 14.75, 10.95, 13.35, 12.43, 10.81,
     14.23, 14.31, 14.69, 13.06, 14.18, 13.35, 12.43, 14.38,
     0,     0,     0,     0,     0,     0,     0,     0,
     0,     0,     0,     0,     0,     0,     0,     0},
    {2.217, 1.968, 1.782, 1.948, 1.555, 1.816, 1.715, 1.541,
     1.835, 1.855, 2.016, 1.775, 1.821, 1.815, 1.713, 1.862,
     0,     0,     0,     0,     0,     0,     0,     0,
     0,     0,     0,     0,     0,     0,     0,     0},
    {1,     1.239, 1.233, 1.203, 1.94, 1.309, 1.592, 1.785, 1.201, 1.169, 1.08,
     1.201, 1.28,  1.138, 1.661, 1,    0,     0,     0,     0,     0,     0,
     0,     0,     0,     0,     0,    0,     0,     0,     0,     0},
    {12.38, 7.835, 6.22,  7.364, 13.3,  5.541, 6.86, 6.056, 15.99, 9.172, 13.08,
     8.809, 12.37, 4.158, 6.732, 10.72, 0,     0,    0,     0,     0,     0,
     0,     0,     0,     0,     0,     0,     0,    0,     0,     0},
    {0.06665, 0.06259, 0.08801, 0.05752, 0.1008,  0.08642, 0.1031, 0.09952,
     0.08328, 0.05468, 0.06668, 0.04685, 0.06749, 0.0657,  0.1105, 0.05781,
     0,       0,       0,       0,       0,       0,       0,      0,
     0,       0,       0,       0,       0,       0,       0,      0},
    {-0.1445, -0.007686, -1.449,  -0.01055, -0.9033,  -1.081,  -1.25,  -1.054,
     -0.2039, -0.08978,  -0.1738, 0.04205,  -0.06929, -0.8856, -1.732, -0.02382,
     0,       0,         0,       0,        0,        0,       0,      0,
     0,       0,         0,       0,        0,        0,       0,      0},
    {10.23, 4.915, 13.86, 4.8,   12.89, 9.734, 11.69, 10.34, 14.94, 6.363, 10.6,
     6.25,  9.584, 8.288, 14.93, 6.861, 0,     0,     0,     0,     0,     0,
     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}};
// suteng end

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
