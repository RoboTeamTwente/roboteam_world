//
// Created by rolf on 10-02-20.
//

#ifndef RTT_BOUNCEPOINT_H
#define RTT_BOUNCEPOINT_H
#include <Eigen/Dense>
struct BouncePoint {
  explicit BouncePoint(Eigen::Vector3d _bouncePos, double _bounceTime):
  bouncePos{_bouncePos},
  bounceTime{_bounceTime}{
  };
  Eigen::Vector3d bouncePos;
  double bounceTime;
};

std::optional<double> calculateBounceTime(double offset, double startVel, double gravity) {
    //We solve (startPos- ballRadius )+startVel*t-0.5 gravity*t^2 = 0for t, taking the highest root we can find.
    double D = startVel*startVel + 2*offset*gravity;
    if (D > 0) {
        // two roots: we return the largest one in time always!
        return (- startVel + sqrt(D))/(- gravity);
    }
    //for both other root cases we return nullopt: they don't make physical sense to be hit
    return std::nullopt;
}
std::optional<BouncePoint> calculateBouncePoint(Eigen::Vector3d startPoint, Eigen::Vector3d startVel, double gravity,
        double ballRadius) {
    std::optional<double> bounceTime=calculateBounceTime(startPoint.z()-ballRadius,startVel.z(),gravity);
    if (!bounceTime){
        return std::nullopt;
    }
    Eigen::Vector3d bouncePos=startPoint+(*bounceTime) * startVel;
    bouncePos.z() = ballRadius;
    BouncePoint bouncePoint(bouncePos,*bounceTime);
    return bouncePoint;
}
#endif //RTT_BOUNCEPOINT_H
