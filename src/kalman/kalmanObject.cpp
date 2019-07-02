//
// Created by kjhertenberg on 13-5-19.
//

#include <roboteam_world/kalman/kalmanObject.h>
#include <roboteam_msgs/DetectionRobot.h>
namespace rtt {

    kalmanObject::kalmanObject() :kalmanObject(INVALID_ID, 1, 1, 1){

    }

    kalmanObject::kalmanObject(uint id, float obsVar, float stateVar, float randVar) {
        //in the future data about them and us might be different so we made different classes
        this->id = id;
        this->observationTimeStamp = -1.0;
        this->invisibleCounter = 0;
        this->visibility = NOT_VISIBLE;
        this->comparisonCount = 0;
        this->orientation = 0;
        this->omega = 0;
        this->cameraId = INVALID_ID;
        this->X.zeros();
        this->Z.zeros();
        this->F = {{1, TIMEDIFF, 0, 0,        0, 0       },
                   {0, 1,        0, 0,        0, 0       },
                   {0, 0,        1, TIMEDIFF, 0, 0       },
                   {0, 0,        0, 1,        0, 0       },
                   {0, 0,        0, 0,        1, TIMEDIFF},
                   {0, 0,        0, 0,        0, 1       }};
        this->H = {{1, 0, 0, 0, 0, 0},
                   {0, 0, 1, 0, 0, 0},
                   {0, 0, 0, 0, 1, 0}};
        this->R = {{obsVar, 0, 0},
                   {0, obsVar, 0},
                   {0, 0, obsVar}};
        this->I.eye();
        this->P = {{stateVar, 0, 0, 0, 0, 0},
                   {0, stateVar, 0, 0, 0, 0},
                   {0, 0, stateVar, 0, 0, 0},
                   {0, 0, 0, stateVar, 0, 0},
                   {0, 0, 0, 0, stateVar, 0},
                   {0, 0, 0, 0, 0, stateVar}};
        arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> tempQ = {{TIMEDIFF, 0, 0},
                                                                 {1       , 0, 0},
                                                                 {0, TIMEDIFF, 0},
                                                                 {0, 1       , 0},
                                                                 {0, 0, TIMEDIFF},
                                                                 {0, 0, 1       }};
        arma::fmat::fixed<OBSERVATIONINDEX, STATEINDEX> tempQ_t = tempQ.t();
        this->Q = tempQ * tempQ_t * randVar;
        this->K.zeros();
    }

    void kalmanObject::kalmanUpdateK() {

        if (this->comparisonCount < MAXCOMPARISONS) {

            /*
             * P = FPF^T+Q
             * S = R + HPH^T
             * K = PHS^-1
             * P = (I-KH)P(I-KH)^T+KRK^T
             */
            arma::fmat::fixed<STATEINDEX, STATEINDEX> F_transpose = this->F.t();
            arma::fmat::fixed<STATEINDEX, STATEINDEX> P_predict = (this->F * this->P * F_transpose) + this->Q;
            arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> H_transpose = this->H.t();
            arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> S = this->R + (this->H * P_predict * H_transpose);
            arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> S_inverse = S.i();
            arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> K_new = P_predict * H_transpose * S_inverse;
            arma::fmat::fixed<OBSERVATIONINDEX, STATEINDEX> K_new_transpose = K_new.t();
            arma::fmat::fixed<STATEINDEX, STATEINDEX> IKH = this->I - K_new * this->H;
            arma::fmat::fixed<STATEINDEX, STATEINDEX> IKH_transpose = IKH.t();
            arma::fmat::fixed<STATEINDEX, STATEINDEX> P_new = IKH * P_predict * IKH_transpose + K_new * this->R * K_new_transpose;

            //See if the K has changed over the iteration, if it hasn't after 100 iterations then stop calculating it
            float K_Diff_Max = (this->K - K_new).max();
            float K_Diff_Min = (this->K - K_new).min();
            int same = 0;
            if ((K_Diff_Max < KMARGIN) and (K_Diff_Min > -KMARGIN)){
                same += 1;
            }

            if (same == STATEINDEX * OBSERVATIONINDEX) {
                this->comparisonCount += 1;
            } else {
                this->comparisonCount = 0;
            }

            for (arma::uword i = 0; i < STATEINDEX; ++i) {
                for (arma::uword j = 0; j < OBSERVATIONINDEX; ++j) {
                    this->K(i, j) = K_new(i, j);
                }
                for (arma::uword k = 0; k < STATEINDEX; ++k) {
                    this->P(i,k) = P_new(i,k);
                }
            }
        }
    }

    void kalmanObject::kalmanUpdateX() {
        // first we update the visibility and check if the ball has been seen the last time
        if (this->invisibleCounter>EXTRAPOLATEDTIME && this->visibility==EXTRAPOLATED){
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

    void kalmanObject::kalmanUpdateRobot(roboteam_msgs::DetectionRobot robot,double timeStamp, uint cameraID){
        Position observation;
        observation.x = robot.pos.x;
        observation.y = robot.pos.y;
        observation.rot = robot.orientation;
        kalmanUpdateZ(observation, timeStamp, cameraID);
    }

    void kalmanObject::kalmanUpdateBall(roboteam_msgs::DetectionBall ball, double timeStamp, uint cameraID){
        Position observation;
        observation.x = ball.pos.x;
        observation.y = ball.pos.y;
        observation.rot = ball.z;
        kalmanUpdateZ(observation, timeStamp, cameraID);
    }

    void kalmanObject::kalmanUpdateZ(Position observation, double timeStamp, uint cameraID) {
        // if we have a ball already and the measurement is too far off we do not trust it.
        if (visibility != NOT_VISIBLE) {
            //HAck
            float errorx = observation.x - this->X(0);
            float errory = observation.y - this->X(2);
            float dist = errorx*errorx + errory*errory;
            if ((this->id == INVALID_ID && dist >= 1.5*1.5) || (this->id != INVALID_ID && dist >= 0.2*0.2) ) {
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
            this->X(4) = observation.rot;
        }
        Position average = calculatePos(observation, cameraID);
        this->cameraId = cameraID;
        this->Z(0) = average.x;
        this->Z(1) = average.y;
        this->Z(2) = calculateRot(average.rot);

        // this is actually the height of the ball, but we are stupid
        this->omega = (average.rot - this->orientation)/(timeStamp - this->observationTimeStamp);
        this->orientation = average.rot;

        this->observationTimeStamp = timeStamp;
        this->invisibleCounter = 0;
        this->visibility = VISIBLE;
    }

    Position kalmanObject::kalmanGetPos() const{
        return {this->X(0), this->X(2), this->X(4)};
    }

    Position kalmanObject::kalmanGetVel() const{
        return {this->X(1), this->X(3), this->X(5)};
    }

    bool kalmanObject::getExistence() const{
        return this->visibility!=NOT_VISIBLE;
    }

    roboteam_msgs::WorldRobot kalmanObject::as_message() const{
        roboteam_msgs::WorldRobot msg;
        Position pos = kalmanGetPos();
        Position vel = kalmanGetVel();
        msg.id = id;
        msg.pos.x = pos.x;
        msg.pos.y = pos.y;
        msg.angle = limitRotation(pos.rot);
        msg.vel.x = vel.x;
        msg.vel.y = vel.y;
        msg.w = vel.rot;
        return msg;
    }

    roboteam_msgs::WorldBall kalmanObject::as_ball_message() {
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

    double kalmanObject::limitRotation(double rotation) const{
        double constRot=fmod(rotation+M_PI, 2*M_PI)-M_PI;
        if (constRot<-M_PI||constRot>=M_PI){
            return -M_PI+std::numeric_limits<float>::epsilon();
        }
        return constRot;
    }

    Position kalmanObject::calculatePos(Position pos, uint camID){
        if (camID == this->cameraId){
            this->pastObservation.clear();
            return pos;
        } else {
            this->pastObservation[camID] = pos;
            Position average = {0, 0, 0};
            for (auto obs : pastObservation) {
                average.x += obs.second.x;
                average.y += obs.second.y;
                average.rot += obs.second.rot;
            }
            float amount = this->pastObservation.size();
            average.y /= amount;
            average.x /= amount;
            average.rot /= amount;
            return average;
        }
    }

    float kalmanObject::calculateRot(float obsRot){
        float rotDiff = this->X(4)-obsRot;
        while (abs(rotDiff)>M_PI){
            if (rotDiff>M_PI){
                obsRot += 2*M_PI;
            } else if (rotDiff<-1*M_PI) {
                obsRot -= 2*M_PI;
            }
            rotDiff = this->X(4)-obsRot;
        }
        return obsRot;
    }

    bool kalmanObject::isVisible() {
        return this->visibility==VISIBLE;
    }

    void kalmanObject::updateVisibility() {
        if (this->invisibleCounter>DISAPPEARTIME){
            this->visibility=NOT_VISIBLE;
        }
        else if (this->invisibleCounter>EXTRAPOLATEDTIME){
            this->visibility=EXTRAPOLATED;
        }
        else{
            this->visibility=VISIBLE;
        }
    }
}