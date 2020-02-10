//
// Created by rolf on 07-02-20.
//

#ifndef RTT_LINEARCHIP6D_H
#define RTT_LINEARCHIP6D_H

#include "data/BallObservation.h"
#include "data/Camera.h"
/**
 * @author Rolf
 * @brief: Key assumptions: Camera's are exactly pointed down!
 *         Advantages: No need to do stupidly difficult coordinate transformations, use of multiple camera detections is possible
 *         Disadvantage: slight loss of precision, but typically very minimal
 */
class LinearChip6D {
    public:
        void solve(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras);
};

#endif //RTT_LINEARCHIP6D_H
