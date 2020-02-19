//
// Created by rolf on 16-02-20.
//

#include "FastDetector.h"
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>
namespace rtt {
bool FastDetector::detectKick(std::vector<BallObservation> observations) {
    assert(observations.size() == 3);
    const BallObservation &first = observations.at(0);
    const BallObservation &second = observations.at(1);
    const BallObservation &third = observations.at(2);
    Vector2 firstPos(first.ball.x(), first.ball.y());
    Vector2 secondPos(second.ball.x(), second.ball.y());
    Vector2 thirdPos(third.ball.x(), third.ball.y());

    Vector2 firstVel = (secondPos - firstPos)/(second.time - first.time);
    Vector2 secondVel = (thirdPos - secondPos)/(third.time - second.time);

    //Check if velocity changed significantly
    constexpr double threshHold = 10000; // mm/s
    if ((secondVel.length() - firstVel.length()) > threshHold) {
        std::cout<<"first"<<std::endl;
        return true;
    }

    constexpr double velThreshHold = 10000; //mm/s
    constexpr double degrees = 20;
    return firstVel.length() > velThreshHold && secondVel.length() > velThreshHold &&
            firstVel.toAngle().angleDiff(secondVel.toAngle()) > toRadians(degrees);

}
}

