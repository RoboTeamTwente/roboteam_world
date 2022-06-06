//
// Created by rolf on 06-06-22.
//

#ifndef RTT_BALLTRAJECTORYSEGMENT_H
#define RTT_BALLTRAJECTORYSEGMENT_H

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LineSegment.h"
#include "roboteam_utils/Time.h"

namespace rtt{
    /**
     * Represents a part of a ball's trajectory where it is under constant deceleration.
     */
    class BallTrajectorySegment {
    private:
        Vector2 startPos;
        Vector2 startVel;
        Vector2 acceleration;
        double acceleration_magnitude;
        Vector2 endPos;
        Time startTime;
        Time endTime;
    public:
        [[nodiscard]] LineSegment path() const;
        [[nodiscard]] Vector2 getVelocity(double time) const;
        [[nodiscard]] Vector2 getPosition(double time) const;
    };
}



#endif //RTT_BALLTRAJECTORYSEGMENT_H
