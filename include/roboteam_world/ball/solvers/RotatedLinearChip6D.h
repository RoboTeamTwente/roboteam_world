//
// Created by rolf on 07-02-20.
//

#ifndef RTT_ROTATEDLINEARCHIP6D_H
#define RTT_ROTATEDLINEARCHIP6D_H
#include <Eigen/Dense>
#include "data/BallObservation.h"
#include "data/Camera.h"
class RotatedLinearChip6D {
    public:
        void solve(std::vector<BallObservation> observations, const std::map<unsigned int, Camera> &cameras);
};

#endif //RTT_ROTATEDLINEARCHIP6D_H
