//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

kalmanFilter::kalmanFilter() {
    //initialise kalman objects
    lastFrameTime = - 1.0;
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        ourBots[i] = kalmanObject(i, posVar, randVar, angleVar);
        theirBots[i] = kalmanObject(i, posVar, randVar, angleVar);
    }
    ball = kalmanObject(INVALID_ID, posVar_ball, randVar_ball, 1);
}

void kalmanFilter::kalmanUpdate() {
    //Updates the Kalman gain (K)
    //Updates the State (X)
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        ourBots[i].kalmanUpdateK();
        ourBots[i].kalmanUpdateX();
        theirBots[i].kalmanUpdateK();
        theirBots[i].kalmanUpdateX();
    }
    ball.kalmanUpdateK();
    ball.kalmanUpdateX();
}

// if we get a new frame we update our observations
void kalmanFilter::newFrame(const roboteam_msgs::DetectionFrame &msg) {
    this->lastFrameTime = msg.t_capture;
    uint cameraID = msg.camera_id;
    for (const roboteam_msgs::DetectionRobot robot : msg.us) {
        ourBots[robot.robot_id].kalmanUpdateRobot(robot, cameraID);
    }
    for (const roboteam_msgs::DetectionRobot robot : msg.them) {
        theirBots[robot.robot_id].kalmanUpdateRobot(robot, cameraID);
    }
    for (const roboteam_msgs::DetectionBall Ball : msg.balls) {
        ball.kalmanUpdateBall(Ball, cameraID);
    }
}

//Creates a world message with the currently observed objects in it
roboteam_msgs::World kalmanFilter::getWorld() {
    roboteam_msgs::World world;
    world.time = lastFrameTime;
    for (const auto& kalmanOurBot : ourBots){
        if (kalmanOurBot.getExistence()){
            world.us.push_back(kalmanOurBot.as_message());
        }
    }
    for (const auto& kalmanTheirBot : theirBots){
        if (kalmanTheirBot.getExistence()){
            world.them.push_back(kalmanTheirBot.as_message());
        }
    }
    world.ball=ball.as_ball_message();
    return world;
}

}