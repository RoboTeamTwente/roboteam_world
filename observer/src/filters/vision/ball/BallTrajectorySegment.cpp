//
// Created by rolf on 06-06-22.
//

#include "observer/filters/vision/ball/BallTrajectorySegment.h"

namespace rtt{
    LineSegment BallTrajectorySegment::path() const {
        return LineSegment(startPos,endPos);
    }
    Vector2 BallTrajectorySegment::getVelocity(double time) const {
            return startVel+startVel.normalize()*acceleration_magnitude*time;
    }

    Vector2 BallTrajectorySegment::getPosition(double time) const {
        return startPos + startVel*time + startVel.normalize()*acceleration_magnitude*time*time*0.5;
    }
}

