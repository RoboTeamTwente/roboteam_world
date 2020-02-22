//
// Created by rolf on 22-2-20.
//

#ifndef RTT_KICKDETECTOR_H
#define RTT_KICKDETECTOR_H

#include <queue>
#include <filters/BallObservation.h>
#include "FastDetector.h"
namespace rtt {
class KickDetector {
    public:
        void addObservation(const BallObservation& observation);
        bool detectKick();
    private:
        std::deque<BallObservation> ballObservations;
        double lastKickTime;
        FastDetector fastDetector;

};

}

#endif //RTT_KICKDETECTOR_H
