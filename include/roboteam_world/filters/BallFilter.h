//
// Created by rolf on 17-11-19.
//

#ifndef RTT_BALLFILTER_H
#define RTT_BALLFILTER_H

#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_proto/WorldBall.pb.h>

#include <utility>
#include "KalmanFilter.h"
#include "CameraFilter.h"

enum State{
    RESTING,
    KICKING,
    SLIPPING,
    ROLLING
};

class BallFilter : public CameraFilter {
    typedef KalmanFilter<4, 2> Kalman;
public:
    //TODO: add documentation
    explicit BallFilter(const proto::SSL_DetectionBall &detectionBall, double detectTime, int cameraID);
    void predict(double time, bool permanentUpdate, bool cameraSwitched);
    void update(double time, bool doLastPredict);;
    void addObservation(const proto::SSL_DetectionBall &detectionBall, double time, int cameraID);
    /**
     * Distance of the state of the filter to a point.
     * @param x xCoordinate (in millimeters!)
     * @param y yCoordinate (in millimeters!)
     * @return Distance from the state to the point (x,y)
     * */
    [[nodiscard]] double distanceTo(double x, double y) const;
    /**
     * Outputs the current filter state in proto format.
     * @return The Proto message associated with the state of the filter
     */
    [[nodiscard]] proto::WorldBall asWorldBall() const;
    /**
     * @return Returns true if the ball has been the last 0.05 seconds.
     */
    [[nodiscard]] bool ballIsVisible() const;
    /**
     * A struct to keep Ball Data and time as one observation.
     */
    struct BallObservation{
        explicit BallObservation(int cameraID,double time,proto::SSL_DetectionBall  detectionBall) :
                cameraID(cameraID),
                time(time),
                ball(std::move(detectionBall))
        {}
        int cameraID;
        double time;
        proto::SSL_DetectionBall ball;
    };
private:
    /**
     * Applies the observation to the kalman Filter at the current time the filter is at.
     * This changes the z and r matrices.
     * Make sure you have predicted until the correct time before calling this!
     * @param detectionBall Ball to be applied to the filter
     */
    void applyObservation(const proto::SSL_DetectionBall &detectionBall, int cameraID);
    /**
     * Initializes the kalman Filter structures
     * @param detectionBall Contains the initial state of the Filter.
     */
    void KalmanInit(const proto::SSL_DetectionBall &detectionBall);
    /**
     * Calculates the velocity of the ball
     * @return Velocity of the ball
     */
    double calculateVelocity() const;
    /**
     * Updates the state of the ball
     */
    void updateState();

    std::unique_ptr<Kalman> kalman = nullptr;
    double lastPredictTime;
    double lastVelocity;
    double kickVelocity;
    State state = RESTING;
    std::vector<BallObservation> observations;
};


#endif //RTT_BALLFILTER_H
