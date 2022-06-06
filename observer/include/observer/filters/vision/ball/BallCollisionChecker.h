// Created by rolf on 06-06-22.
//

#ifndef RTT_BALLCOLLISIONCHECKER_H
#define RTT_BALLCOLLISIONCHECKER_H


#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Time.h"

/**
 * This file contains functions which predict for a short time ahead if a ball will collide with a wall/robot,
 * and if it does collide, predicts where and how the ball will collide
 */
namespace rtt{
    enum class BallCollisionType{
        OUTER_WALL, //Walls on the outside of the field
        GOAL_OUTSIDE, //Outside of the goal (distinction is made because materials may be different)
        GOAL_POST, // Goal posts (small surface facing towards the field only)
        GOAL_INSIDE, // Usually has better damping
        ROBOT_HULL, // Hull of a robot (e.g. against one of the wheels or the back/side)
        ROBOT_FRONT //Front of a robot
    };

    /**
     * This struct is only used internally by the collision checker
     */
    struct PreliminaryCollisionResult{
        Vector2 normalDir; //Line on which the normal lies, may point 180 degrees in the wrong direction!
        double distanceFraction; //Fraction of the distance on which the collision point lies
        BallCollisionType type;
    };

    struct BallCollision{
        Vector2 position;
        Vector2 inVelocity;
        Vector2 outVelocity;
        double dt; //the time relative to a given BallTrajectorySegment this collision takes place
        Time collisionTime;
        BallCollisionType type;
        //info on the robot which it collided against
        int robotID = -1;
        bool robotIsBlue = false;
    };

}


#endif //RTT_BALLCOLLISIONCHECKER_H
