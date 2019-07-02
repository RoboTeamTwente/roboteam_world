//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H


#include "kalmanObject.h"
#include "roboteam_utils/Position.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/World.h"

namespace rtt {

    //This class is a manager for the different Kalman object classes
    class kalmanFilter {
        private:
            double lastFrameTime;

    public:
        kalmanFilter();

        void kalmanUpdate();

        void newFrame(const roboteam_msgs::DetectionFrame& msg);

        roboteam_msgs::World getWorld();

        kalmanObject theirBots[BOTCOUNT];
        kalmanObject ourBots[BOTCOUNT];
        kalmanObject ball;
    };
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
