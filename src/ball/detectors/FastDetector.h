//
// Created by rolf on 16-02-20.
//

#ifndef RTT_FASTDETECTOR_H
#define RTT_FASTDETECTOR_H

#include <vector>
#include <filters/BallObservation.h>
namespace rtt{
class FastDetector {
    public:
        bool detectKick(std::vector<BallObservation> observations);
        bool noiseRect(std::vector<BallObservation> observations);
        double RELUFunc(double value,double initial, double threshHold, double slope);
};
}


#endif //RTT_FASTDETECTOR_H
