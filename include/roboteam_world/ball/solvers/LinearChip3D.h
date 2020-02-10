//
// Created by rolf on 08-02-20.
//

#ifndef RTT_LINEARCHIP3D_H
#define RTT_LINEARCHIP3D_H

#include "data/BallObservation.h"
#include "data/Camera.h"

class LinearChip3D {
    public:
        double solve(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras,Eigen::Vector2d kickPos,double timeOffset,bool print=false);
        void binSearch(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras, Eigen::Vector2d kickPos);

};

#endif //RTT_LINEARCHIP3D_H
