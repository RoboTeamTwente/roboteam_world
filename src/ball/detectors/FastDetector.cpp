//
// Created by rolf on 16-02-20.
//

#include "FastDetector.h"
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Polygon.h>
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

    std::cout << "pos" << thirdPos << "dt: " << (third.time - second.time) << std::endl;
    //Check if velocity changed significantly
    constexpr double threshHold = 1000; // mm/s
    if ((secondVel.length() - firstVel.length()) > threshHold) {
        std::cout << "first" << std::endl;
        return true;
    }

    constexpr double velThreshHold = 1000; //mm/s
    constexpr double degrees = 20;
    return firstVel.length() > velThreshHold && secondVel.length() > velThreshHold &&
            firstVel.toAngle().angleDiff(secondVel.toAngle()) > toRadians(degrees);
}
bool FastDetector::noiseRect(std::vector<BallObservation> observations) {
    assert(observations.size() == 3);
    const BallObservation &first = observations.at(0);
    const BallObservation &second = observations.at(1);
    const BallObservation &third = observations.at(2);
    assert(third.time > first.time);
    Vector2 firstPos(first.ball.x(), first.ball.y());
    Vector2 secondPos(second.ball.x(), second.ball.y());
    Vector2 thirdPos(third.ball.x(), third.ball.y());

    Vector2 oldVel = (secondPos - firstPos)/(second.time - first.time);
    Vector2 newVel = (thirdPos - secondPos)/(third.time - second.time);

    //noise rectangle; rectangle within the velocity state diagram where the velocity is still considered to be close enough to be kicked
    //We consider the linear direction (direction the ball is rolling in) and the direction orthogonal to that linear direction
    //We use RELU like functions that look roughly like this to determine what is an acceptable noise level/deceleration for something to be seen as a ball contact
    //                                 /
    //                                /
    //                          -----/
    // height of (-) = base
    // x at which - changes to / = treshhold
    // slope of / = slope
    constexpr double baseLinear = 100; //mm/s probably needs to be higher
    constexpr double baseOrthogonal = 100; //mm/s

    constexpr double linearThreshold = 100; //mm/s
    constexpr double orthogonalThreshold = 100; //m/s

    constexpr double orthogonalSlope = 0.1;
    constexpr double linearSlope = 0.15;

    //we assume the ball is also allowed to decellerate at max this rate.
    constexpr double maxDeceleration = 5000; // m/s^2

    double oldLength = oldVel.length();
    double linearDeviation = RELUFunc(oldLength, baseLinear, linearThreshold, linearSlope);
    double orthogonalDeviation = RELUFunc(oldLength, baseOrthogonal, orthogonalThreshold, orthogonalSlope);

//    std::cout << "vel: " << newVel.length() << "old: " << oldVel.length() << "lin dev: " << linearDeviation
//              << "orth dev: " << orthogonalDeviation << std::endl;

    Vector2 proj = newVel.project2(oldVel);
    double actualLinDev = proj.length();
    double actualOrthDev = sqrt(newVel.length2() - proj.length2());
    actualLinDev = actualLinDev - oldVel.length();

    std::cout<<newVel.toAngle()<< " "<<newVel.length()<<std::endl;
    std::cout<< linearDeviation <<" "<< actualLinDev<<" "<< -(linearDeviation+maxDeceleration*(third.time-second.time))<<std::endl;
    std::cout<< orthogonalDeviation <<" "<< actualOrthDev<<" "<< -orthogonalDeviation <<std::endl;
    std::cout<< "_________________"<<std::endl;
    if (actualLinDev < linearDeviation && actualLinDev >-(linearDeviation+maxDeceleration*(third.time-second.time)) &&
            actualOrthDev < orthogonalDeviation && actualOrthDev > -orthogonalDeviation
            ) {
        return false;
    }
    return true;
    // defining the rectangle around the velocity for which the deviation is acceptable (is always faced towards the origin)
    Vector2 centerLow = oldVel - oldVel.stretchToLength(linearDeviation + maxDeceleration*(third.time - second.time));
    Vector2 centerHigh = oldVel + oldVel.stretchToLength(linearDeviation);
    Vector2 orthogonalVector = Vector2(orthogonalDeviation, 0).rotate(oldVel.angle() + M_PI_2);
    Vector2 point1 = centerLow + orthogonalVector;
    Vector2 point2 = centerLow - orthogonalVector;
    Vector2 point3 = centerHigh - orthogonalVector;
    Vector2 point4 = centerHigh + orthogonalVector;
    Polygon rectangle({point1, point2, point3, point4});
    // check the noise rectangle
    if (rectangle.contains(newVel)) {
        // ball does not collide and is rolling roughly straight
//        collidesNow = false;
//        ballStraightTicks ++;
//        kickedNow = false;
        return false;
    }
    else {
//        ballStraightTicks = 0;
        // kicked or collided
        if (newVel.length() > (oldVel.length() + linearDeviation)) {
//            kickedNow = true;
            return true;
        }
        else {
//            collidesNow = true;
            return true;
        }
    }
}
double FastDetector::RELUFunc(double value, double base, double threshHold, double slope) {
    if (value < threshHold) {
        return base;
    }
    else {
        return base + slope*(value - threshHold);
    }
}

}

