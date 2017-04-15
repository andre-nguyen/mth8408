//
// Created by andre on 4/15/17.
//

#ifndef SEGMENTTIMEOPTIMIZER_H
#define SEGMENTTIMEOPTIMIZER_H

#include <alglib/optimization.h>

namespace an_min_snap_traj {
    class SegmentTimeOptimizer {
    public:
        SegmentTimeOptimizer(TrajectoryGenerator* tg);

    private:
        TrajectoryGenerator* tg_;
    };
}

#endif //SEGMENTTIMEOPTIMIZER_H
