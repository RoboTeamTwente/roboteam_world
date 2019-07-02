//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanBall.h"

namespace rtt {
kalmanBall::kalmanBall() :kalmanObject(INVALID_ID, posVar_ball, stateVar_ball, randVar_ball){
}

void kalmanBall::kalmanUpdateBall(roboteam_msgs::DetectionBall ball, double timeStamp, uint cameraID){
    Position observation;
    observation.x = ball.pos.x;
    observation.y = ball.pos.y;
    observation.rot = ball.z;
    kalmanUpdateZb(observation, timeStamp, cameraID);
}


void kalmanBall::kalmanUpdateZb(Position observation, double timeStamp, uint cameraID) {
    // if we have a ball already and the measurement is too far off we do not trust it.
    if (visibility != NOT_VISIBLE) {
        //HAck
        float errorx = observation.x - this->X(0);
        float errory = observation.y - this->X(2);
        if (errorx*errorx + errory*errory >= 2) {
            return;
        }
    }
    else {
        // we found a new ball so we are resetting the state. We assume it's velocity is 0.
        std::cout<<"Jumping the ball"<<std::endl;
        this->pastObservation.clear();
        this->X.zeros();
        this->X(0) = observation.x;
        this->X(2) = observation.y;
    }
    Position average = calculatePos(observation, cameraID);
    this->cameraId = cameraID;
    this->Z(0) = average.x;
    this->Z(1) = average.y;

    // this is actually the height of the ball, but we are stupid
    this->omega = (average.rot - this->orientation)/(timeStamp - this->observationTimeStamp);
    this->orientation = average.rot;

    this->observationTimeStamp = timeStamp;
    this->invisibleCounter = 0;
    this->visibility = VISIBLE;
}

roboteam_msgs::WorldBall kalmanBall::as_ball_message() {
    //Same as the KalmanObject function but then for ball message
    roboteam_msgs::WorldBall msg;
    Position pos = kalmanGetPos();
    Position vel = kalmanGetVel();
    // since the balls z axis is being kept in the third place of the vector it is the 'rotation' here
    msg.existence = 1;
    msg.visible = isVisible();
    msg.pos.x = pos.x;
    msg.pos.y = pos.y;

    msg.z = pos.rot;
    msg.vel.x = vel.x;
    msg.vel.y = vel.y;
    msg.z_vel = vel.rot;
    return msg;
}

void kalmanBall::kalmanUpdateX() {
    // first we update the visibility and check if the ball has been seen the last time
    if (this->invisibleCounter>BALLEXTRAPOLATEDTIME && this->visibility==EXTRAPOLATED){
        std::cout<<"Invisible ball! Not moving it anymore. "<<std::endl;
    }
    updateVisibility();

    // X_predict = FX_current
    // Y = Z - HX_predict
    // X_new = X_predict + Ky

    if (this->visibility!=NOT_VISIBLE){
        arma::fvec::fixed<STATEINDEX> X_predict = this->F*this->X;
        arma::fmat::fixed<OBSERVATIONINDEX, 1> Y = this->Z - (this->H*X_predict);
        if (this->invisibleCounter>0){ // we only use the observation if we actually received one.
            Y.zeros();
        }
        arma::fvec::fixed<STATEINDEX> X_new = X_predict + (this->K * Y);

        for (arma::uword i = 0; i < STATEINDEX; ++ i) {
            this->X(i) = X_new(i);
        }
    }
    this->invisibleCounter += 1; // we update the amount of loops which we did without seeing the ball (this is reset to 0 if the ball is seen again).
}

bool kalmanBall::isVisible() {
    return this->visibility==VISIBLE;
}

void kalmanBall::updateVisibility() {
    if (this->invisibleCounter>BALLDISAPPEARTIME){
        this->visibility=NOT_VISIBLE;
    }
    else if (this->invisibleCounter>BALLEXTRAPOLATEDTIME){
        this->visibility=EXTRAPOLATED;
    }
    else{
        this->visibility=VISIBLE;
    }
}
}