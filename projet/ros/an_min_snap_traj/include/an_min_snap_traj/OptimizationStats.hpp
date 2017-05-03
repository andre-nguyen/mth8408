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

/**
 * @author  Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 * @file    OptimizationStats.hpp
 * @details Just a little object to store optimization stats
 */

#ifndef AN_MIN_SNAP_TRAJ_OPTIMIZATIONSTATS_H
#define AN_MIN_SNAP_TRAJ_OPTIMIZATIONSTATS_H

namespace an_min_snap_traj {
    class OptimizationStats {
    public:
        OptimizationStats() {};
        OptimizationStats(int iters, double obj_val): iters_(iters), obj_val_
                (obj_val) {};

        int getNumIters() { return iters_; }
        double getObjVal() { return obj_val_; }
        bool getResult() {return success_;}
    private:
        friend class IpoptAdapter;
        int iters_;
        double obj_val_;
        bool success_;
    };
}

#endif //AN_MIN_SNAP_TRAJ_OPTIMIZATIONSTATS_H
