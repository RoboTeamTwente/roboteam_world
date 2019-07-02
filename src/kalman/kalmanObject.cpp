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
        this->exists = false;
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

        this->invisibleCounter += 1;
        if (this->invisibleCounter> DISAPPEARTIME&&this->exists){
            std::cout << "Removing bot: " << this->id<< std::endl;
        }
        if (this->invisibleCounter > DISAPPEARTIME || !this->exists) {
            this->exists = false;
        } else {
            // X_predict = FX_current
            // Y = Z - HX_predict
            // X_new = X_predict + Ky

            arma::fvec::fixed<STATEINDEX> X_predict = this->F * this->X;

            arma::fmat::fixed<OBSERVATIONINDEX, 1> Y = this->Z - (this->H * X_predict);

            arma::fvec::fixed<STATEINDEX> X_new = X_predict + (this->K * Y);

            for (arma::uword i = 0; i < STATEINDEX; ++i) {
                this->X(i) = X_new(i);
            }

        }
    }

    void kalmanObject::kalmanUpdateRobot(roboteam_msgs::DetectionRobot robot,double timeStamp, uint cameraID){
        Position observation;
        observation.x = robot.pos.x;
        observation.y = robot.pos.y;
        observation.rot = robot.orientation;
        kalmanUpdateZ(observation, timeStamp, cameraID);
    }

    void kalmanObject::kalmanUpdateZ(Position observation, double timeStamp, uint cameraID) {
        //if the new data is a certain distance from the old/predicted data, it's considered a ghost and ignored
        if (this->exists){
            float errorx = observation.x-this->X(0);
            float errory = observation.y-this->X(2);
            if (errorx*errorx+errory*errory >= 0.2*0.2){
                return;
            }
        }
        //if the object comes into being, make the observation it's state, (to prevent jumping)
        if (!this->exists){
            std::cout<<"Adding bot: "<<this->id<<std::endl;
            this->pastObservation.clear();
            this->X(0) = observation.x;
            this->X(2) = observation.y;
            this->X(4) = observation.rot;
        }
        Position average = calculatePos(observation, cameraID);
        this->cameraId = cameraID;
        this->Z(0) = average.x;
        this->Z(1) = average.y;
        this->Z(2) = calculateRot(average.rot);
        this->omega = (average.rot - this->orientation)/(timeStamp-this->observationTimeStamp);
        this->orientation = average.rot;
        this->observationTimeStamp = timeStamp;
        this->invisibleCounter = 0;
        this->exists = true;
    }

    Position kalmanObject::kalmanGetPos() const{
        return {this->X(0), this->X(2), this->X(4)};
    }

    Position kalmanObject::kalmanGetVel() const{
        return {this->X(1), this->X(3), this->X(5)};
    }

    float kalmanObject::getK(){
        return this->K(0, 0);
    }

    bool kalmanObject::getExistence() const{
        return this->exists;
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


}