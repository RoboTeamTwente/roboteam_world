//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

    void kalmanInit() {
        for (uint i = 0; i < 32; ++i) {
            robotlist[i] = kalmanObject(i);
        }

    }

    void kalmanUpdate() {
        for (int i = 0; i < 32; ++i) {
            robotlist[i].kalmanUpdateK();
            robotlist[i].kalmanUpdateX();
        }
    }

    void newFrame(const roboteam_msgs::DetectionFrame msg) {
        double timeCapture = msg.t_capture;
        for (const roboteam_msgs::DetectionRobot robot : msg.them) {
            robotlist[robot.robot_id].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        }
        for (const roboteam_msgs::DetectionRobot robot : msg.us) {
            robotlist[robot.robot_id].kalmanUpdateZ(robot.pos.x, robot.pos.y, robot.orientation, timeCapture);
        }
    }

    Position getStates(uint id) {
        return robotlist[id].kalmanGetState();
    }

    float getK(uint id) {
        return robotlist[id].getK();
    }

    void setZ(uint id, float x, float y, float z, double timestamp){
        robotlist[id].kalmanUpdateZ(x, y, z, timestamp);
    }

}