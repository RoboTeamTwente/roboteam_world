//
// Created by rolf on 22-2-20.
//

#include <ball/detectors/KickDetector.h>
#include "ball/detectors/KickDetector.h"
namespace rtt{
void KickDetector::addObservation(const BallObservation& observation) {
    ballObservations.push_back(observation);
    if (ballObservations.size()>300){ //TODO: choose max number
        ballObservations.pop_front();
    }

}
bool KickDetector::detectKick() {
    if (ballObservations.size()<4){
        return false;
    }
    bool detectedKick=fastDetector.detectKick(std::vector<BallObservation>(ballObservations.end()-3,ballObservations.end()));
    if (detectedKick){
        bool previousWasKick = fastDetector.detectKick(std::vector<BallObservation>(ballObservations.end()-4,ballObservations.end()-1));
        return !previousWasKick;
    }
    return false;
}
}