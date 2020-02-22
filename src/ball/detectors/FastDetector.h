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
        bool detectKick(const std::vector<BallObservation> &observations);
};
}


#endif //RTT_FASTDETECTOR_H
