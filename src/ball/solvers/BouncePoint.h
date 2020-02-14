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

std::optional<double> calculateBounceTime(double offset, double startVel, double gravity);
std::optional<BouncePoint> calculateBouncePoint(Eigen::Vector3d startPoint, Eigen::Vector3d startVel, double gravity,
        double ballRadius);
#endif //RTT_BOUNCEPOINT_H
