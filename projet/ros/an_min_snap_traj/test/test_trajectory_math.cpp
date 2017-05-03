/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2017 Mobile Robotics and Autonomous Systems Laboratory (MRASL),
 * Polytechnique Montreal. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <an_min_snap_traj/TrajectoryMath.hpp>

using namespace an_min_snap_traj;
using namespace Eigen;

TEST(traj_math, polyval0){
    Vector3d v;
    v << 3, 2, 1;
    double answer = polyval(v, 5.0);
    ASSERT_TRUE(answer == 86);
    answer = polyval(v, 7);
    ASSERT_TRUE(answer == 162);
    answer = polyval(v, 9);
    ASSERT_TRUE(answer == 262);
}

TEST(traj_math, polyder0){
    VectorXd v(7);
    v << 1, 1, 1, 1, 1, 1, 1;
    VectorXd vdev(6);
    vdev << 6, 5, 4, 3, 2, 1;
    auto answer = polyder(v);

    ASSERT_TRUE(answer.isApprox(vdev));

    answer = polyder(vdev);
    VectorXd vans(5);
    vans << 30, 20, 12, 6, 2;
    ASSERT_TRUE(answer.isApprox(vans));
}

TEST(traj_math, rot90){
    MatrixXd I(3,3);
    I << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    auto answer = rot90(I);
    MatrixXd expected(3,3);
    expected << 3, 6, 9, 2, 5, 8, 1, 4, 7;
    ASSERT_TRUE(answer.isApprox(expected));

    answer = rot90(I, 2);
    expected << 9, 8, 7, 6, 5, 4, 3, 2, 1;
    ASSERT_TRUE(answer.isApprox(expected));
}

TEST(traj_math, getCoefficients){
    MatrixXd expected(5, 7);
    expected <<     1     ,1    ,1   ,1  ,1  ,1  ,1
                    ,6    ,5    ,4   ,3  ,2  ,1  ,0
                    ,30   ,20   ,12  ,6  ,2  ,0  ,0
                    ,120  ,60   ,24  ,6  ,0  ,0  ,0
                    ,360  ,120  ,24  ,0  ,0  ,0  ,0;
    
    auto answer = genCoefficientMatrix(6, 4);
    ASSERT_TRUE(answer.isApprox(expected));
}