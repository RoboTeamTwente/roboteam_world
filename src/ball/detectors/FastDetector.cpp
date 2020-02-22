//
// Created by rolf on 16-02-20.
//

#include "FastDetector.h"
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Polygon.h>
namespace rtt {
bool FastDetector::detectKick(const std::vector<BallObservation> &observations) {
    assert(observations.size() == 3);
    const BallObservation &first = observations.at(0);
    const BallObservation &second = observations.at(1);
    const BallObservation &third = observations.at(2);
    assert(first.time<third.time);     //The observations should be ordered the right way

    Vector2 firstPos(first.ball.x(), first.ball.y());
    Vector2 secondPos(second.ball.x(), second.ball.y());
    Vector2 thirdPos(third.ball.x(), third.ball.y());

    Vector2 firstVel = (secondPos - firstPos)/(second.time - first.time);
    Vector2 secondVel = (thirdPos - secondPos)/(third.time - second.time);

    //Check if velocity changed significantly
    double speedDiff=(secondVel.length() - firstVel.length());
    // absolute difference in angle
    double angleDiff=abs(firstVel.toAngle().shortestAngleDiff(secondVel.toAngle()));

    constexpr double threshHold = 1000; // mm/s
    constexpr double rollingDeviation = 10; // degrees
    // Due to timing inaccuracies some camera's may give a wrong time stamp to a detection or be slightly inaccurate.
    // If the ball is rolling fast this results in invalid speed estimations.
    // If the ball is still on the same line (within 10 radians), and was already rolling faster than the threshhold
    // we still assume that the ball is NOT kicked because of this. Otherwise speed oscilations will lead to invalid kick detections
    // Perhaps we can use this in the future to mark detection frames as having an 'inaccurate' time stamp
    if (speedDiff > threshHold && (angleDiff > toRadians(rollingDeviation) || firstVel.length() < threshHold))  {
        return true;
    }
    constexpr double velThreshHold = 1000; //mm/s
    constexpr double kickDeviation = 20; //degrees
    //This is more effective at catching redirects and especially rebounds.
    return firstVel.length() > velThreshHold && secondVel.length() > velThreshHold &&
            angleDiff > toRadians(kickDeviation);
}

}

