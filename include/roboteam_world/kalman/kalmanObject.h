//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANOBJECT_H
#define ROBOTEAM_WORLD_KALMANOBJECT_H

#include "armadillo"
#include "roboteam_utils/Position.h"
#include "constantsK.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/WorldBall.h"

namespace rtt {

    //class for objects who's data goes trough the kalman filter
    //Based on: https://en.wikipedia.org/wiki/Kalman_filter
class kalmanObject {

    protected:
        uint id; //Id of the object if applicable
        double observationTimeStamp; //Time of last observed data used to make sure old data doesn't replace new data
        int invisibleCounter; //count the ticks between observations, after a certain time the object doesn't exist anymore
        visState visibility;
        int comparisonCount; //time the iteration of P and K where they are the same
        float orientation; //currently the filter only filters X and Y, du to the coordinate system
        double omega; //""
        uint cameraId;
        std::map<int, Position> pastObservation;

        // The key matrices (fixed size to prevent side effects)
        arma::fvec::fixed<STATEINDEX> X; //Constains the state of the robot
        arma::fvec::fixed<OBSERVATIONINDEX> Z; //contains the lates observations
        arma::fmat::fixed<STATEINDEX, STATEINDEX> F; //dynamics how current state should change tot the next state
        arma::fmat::fixed<OBSERVATIONINDEX, STATEINDEX> H; //How the state should project to the observation
        arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> R; //The covariance of the observation noise
        arma::fmat::fixed<STATEINDEX, STATEINDEX> I; //Identity matrix
        arma::fmat::fixed<STATEINDEX, STATEINDEX> P; //A measure of the estimated accuracy of the state estimate
        arma::fmat::fixed<STATEINDEX, STATEINDEX> Q; // The covariance of the process noise (random forces)
        arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> K; //Kalman gain, found based on the variances inputted.

    public:

        kalmanObject();

        kalmanObject(uint id, float obsVar, float stateVar, float randVar);

        //updates the K till it doesn't change anymore (with wrong variance it can osscilate or grow exponetially)
        void kalmanUpdateK();

        //If the object exists, updates the state
        virtual void kalmanUpdateX();

        void kalmanUpdateRobot(roboteam_msgs::DetectionRobot robot,double timeStamp, uint cameraID);

        void kalmanUpdateBall(roboteam_msgs::DetectionBall ball, double timeStamp, uint cameraID);

        //if the data is more recent than the current data, import the new observation data
        void kalmanUpdateZ(Position observation,double timeStamp, uint cameraID);

        //Get X,Y and Orientation
        Position kalmanGetPos() const;

        //Get X_vel, Y_vel and omega
        Position kalmanGetVel() const;

        //Does the object exist
        bool getExistence() const;

        //Create a message, by default it's a robot message (the ball object overrides this)
        virtual roboteam_msgs::WorldRobot as_message() const;

        //Same as the KalmanObject function but then for ball message
        roboteam_msgs::WorldBall as_ball_message();

        double limitRotation(double rotation) const;

        //Averages the data of multiple cameras
        Position calculatePos(Position pos, uint camID);

        //Ensures that the observed orientation is neveroat rotDiff = this->X(4)-obsRot; further away than pi from the state orientation
        float calculateRot(float obsRot);

        bool isVisible();

        void updateVisibility();

};

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
