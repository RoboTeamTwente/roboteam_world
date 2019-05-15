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

    class kalmanFilter {

    public:
        kalmanFilter();

        void kalmanUpdate();

        void newFrame(const roboteam_msgs::DetectionFrame msg);

        kalmanObject kalmanlist[32];

        Position getPos(uint id);

        Position getVel(uint id);

        float getK(uint id);

        void setZ(uint id, float x, float y, float z, double timestamp);

        bool getExistance(uint id);

        roboteam_msgs::WorldRobot getRobot(uint id);

        roboteam_msgs::World getWorld();

        uint magicnumber = 32;

    };
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
