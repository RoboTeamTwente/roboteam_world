//
// Created by rolf on 17-11-19.
//

#include "filters/BallFilter.h"
#include "Scaling.h"

BallFilter::BallFilter(const proto::SSL_DetectionBall &detectionBall, double detectTime, int cameraID) :
        CameraFilter(detectTime, cameraID),
        lastPredictTime{detectTime} {
    KalmanInit(detectionBall);
}
void BallFilter::KalmanInit(const proto::SSL_DetectionBall &detectionBall) {
    // SSL units are in mm, we do everything in SI units.
    double x = mmToM(detectionBall.x());//m
    double y = mmToM(detectionBall.y());//m
    Kalman::Vector startState = {
            x, y, 0, 0
    };

    Kalman::Matrix startCov;
    startCov.eye();
    //initial noise estimates
    const double startPosNoise = 0.05;
    startCov.at(0, 0) = startPosNoise;//m noise in x
    startCov.at(1, 1) = startPosNoise;//m noise in y

    kalman = std::make_unique<Kalman>(startState, startCov);

    kalman->H.eye();     // Our observations are simply what we see.


}
void BallFilter::applyObservation(const proto::SSL_DetectionBall &detectionBall, int cameraID) {
    Kalman::VectorO observation = {mmToM(detectionBall.x()), mmToM(detectionBall.y())};
    //TODO: do things with the other ball fields (pixel pos, area)
    kalman->z = observation;

    //Observations which are not from the main camera are added but are seen as much more noisy
    const double posVar = 0.02; //variance TODO: tune these 2
    const double posVarOtherCamera = 0.05;
    kalman->R.zeros();
    if (cameraID == mainCamera) {
        kalman->R.at(0, 0) = posVar;
        kalman->R.at(1, 1) = posVar;
    } else {
        kalman->R.at(0, 0) = posVarOtherCamera;
        kalman->R.at(1, 1) = posVarOtherCamera;
    }
    kalman->update();
}
proto::WorldBall BallFilter::asWorldBall() const {
    proto::WorldBall msg;
    const Kalman::Vector &state = kalman->state();
    msg.mutable_pos()->set_x(state[0]);
    msg.mutable_pos()->set_y(state[1]);
    msg.mutable_vel()->set_x(state[2]);
    msg.mutable_vel()->set_y(state[3]);
    msg.set_visible(ballIsVisible());
    //TODO: add height filter here, and actually set the z, z_vel, area fields
    return msg;
}
double BallFilter::distanceTo(double x, double y) const {
    const Kalman::Vector &state = kalman->state();
    double dx = state[0] - mmToM(x);
    double dy = state[1] - mmToM(y);
    return sqrt(dx * dx + dy * dy);
}
void BallFilter::predict(double time, bool permanentUpdate, bool cameraSwitched) {
    double dt = time - lastUpdateTime;
    // forward model:
    kalman->F.eye();
    kalman->F.at(0, 2) = dt;
    kalman->F.at(1, 3) = dt;

    // Two phase forward model
    double acceleration = determineAcceleration();
    double direction = atan2(kalman->state()[3], kalman->state()[2]);

    kalman->F.at(2, 2) = 1 + cos(direction) * acceleration * dt;
    kalman->F.at(3, 3) = 1 + sin(direction) * acceleration * dt;

    //Set B
    kalman->B = kalman->F;
    //Set u (we have no control input at the moment)
    kalman->u.zeros();

    //Set Q matrix
    const float posNoise = 0.1;//TODO: tune
    Kalman::MatrixO G;
    G.zeros();
    G.at(0, 0) = dt * posNoise;
    G.at(0, 2) = 1 * posNoise;
    G.at(1, 1) = dt * posNoise;
    G.at(1, 3) = 1 * posNoise;
    if (cameraSwitched){
        G.at(0,0)+=0.05;
        G.at(1,1)+=0.05;
    }
    kalman->Q = G.t() * G ;

    kalman->predict(permanentUpdate);
    lastPredictTime = time;
    if (permanentUpdate) {
        lastUpdateTime = time;
    }
}
bool compareObservation(const BallFilter::BallObservation &a, const BallFilter::BallObservation &b) {
    return (a.time < b.time);
}
void BallFilter::update(double time, bool doLastPredict) {

    std::sort(observations.begin(), observations.end(),
              compareObservation); //First sort the observations in time increasing order
    auto it = observations.begin();
    while (it != observations.end()) {
        auto observation = (*it);
        //the observation is either too old (we already updated the ball) or too new and we don't need it yet.
        if (observation.time < lastUpdateTime) {
            observations.erase(it);
            continue;
        }
        if (observation.time > time) {
            //relevant update, but we don't need the info yet so we skip it.
            ++it;
            continue;
        }
        // We first predict the ball, and then apply the observation to calculate errors/offsets.
        bool cameraSwitched=switchCamera(observation.cameraID, observation.time);
        predict(observation.time, true,cameraSwitched);
        applyObservation(observation.ball, observation.cameraID);
        observations.erase(it);
    }
    if (doLastPredict) {
        predict(time, false,false);
    }

}
void BallFilter::addObservation(const proto::SSL_DetectionBall &detectionBall, double time, int cameraID) {
    observations.emplace_back(BallObservation(cameraID, time, detectionBall));
}
bool BallFilter::ballIsVisible() const {
    //If we extrapolated the ball for longer than 0.05 seconds we mark it not visible
    return (lastPredictTime - lastUpdateTime) < 0.05;
}

double BallFilter::calculateVelocity() const {
    double xVel = kalman->state()[2];
    double yVel = kalman->state()[3];
    return sqrt(xVel * xVel + yVel * yVel);
}

double BallFilter::determineAcceleration() {
    //TODO: Estimate values with experiments
    const double accelerationSlide = -2.5; // First phase
    const double accelerationRoll = -0.3; // Second phase

    // Detect when ball is kicked
    const double thresholdBallKicked = 1.0; //TODO: Tune when the ball should be considered kicked
    double velocity = calculateVelocity();
    if (velocity - lastVelocity > thresholdBallKicked) {
        ballKicked = true;
        kickVelocity = velocity;
    }

    // Determine the initial velocity of the kicked ball
    if (ballKicked) {
        if (velocity > kickVelocity) {
            kickVelocity = velocity;
        } else {
            ballKicked = false;
        }
    }

    lastVelocity = velocity;

    // Determines the phase
    const double switchRatio = 0.6; //TODO: Estimate with experiments
    if (velocity > switchRatio * kickVelocity) {
        return accelerationSlide;
    } else {
        return accelerationRoll;
    }
}