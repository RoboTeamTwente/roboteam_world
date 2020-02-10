//
// Created by rolf on 07-02-20.
//

#ifndef RTT_ROTATEDLINEARCHIP6D_H
#define RTT_ROTATEDLINEARCHIP6D_H
#include <Eigen/Dense>
#include "data/BallObservation.h"
#include "data/Camera.h"
/**
 * @author Rolf
 * @brief: Key assumptions: Only one camera which can have some rotation.
 * Is more expensive to compute but works better if a camera is not pointed almost exactly downwards.
 * The error w.r.t to the LinearChip6D class will mostly be in z_0 and v_z if the camera is 'almost' vertically downwards
 */
class RotatedLinearChip6D {
    public:
        void solve(std::vector<BallObservation> observations, const std::map<unsigned int, Camera> &cameras);
};

#endif //RTT_ROTATEDLINEARCHIP6D_H
