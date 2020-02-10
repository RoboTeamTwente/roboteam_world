//
// Created by rolf on 08-02-20.
//

#ifndef RTT_LINEARCHIP5D_H
#define RTT_LINEARCHIP5D_H

#include "data/BallObservation.h"
#include "data/Camera.h"

class LinearChip5D {
    public:
        double solve(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras, double timeOffset, bool print=false);
        void binSearch(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras);
};

#endif //RTT_LINEARCHIP5D_H
