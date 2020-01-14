#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H

#include <filters/RobotFilter.h>
#include <filters/BallFilter.h>
#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"

namespace world {

    /**
     * @author Rolf van der Hulst
     * @date November 2019
     * @brief class that tracks a world. The function of this class is to determine when to create and delete filters,
     * and to pass the relevant incoming information to the relevant filter(s)
     */
    class WorldFilter {
    public:
        WorldFilter();
        void addFrame(const proto::SSL_DetectionFrame &msg);
        void addCamera(const proto::SSL_GeometryCameraCalibration &cameraCalibration);
        proto::World getWorld(double time);
    private:
        void update(double time, bool extrapolateLastStep);
        typedef std::map<int, std::vector<std::unique_ptr<RobotFilter>>> robotMap;
        static const std::unique_ptr<RobotFilter> &bestFilter(const std::vector<std::unique_ptr<RobotFilter>> &filters);
        static const std::unique_ptr<BallFilter> &bestFilter(const std::vector<std::unique_ptr<BallFilter>> &filters);
        typedef std::map<int, proto::SSL_GeometryCameraCalibration> camMap;
        robotMap blueBots;
        robotMap yellowBots;
        camMap cameras;
        std::vector<std::unique_ptr<BallFilter>> balls;
        static void updateRobots(robotMap &robots, double time, bool extrapolateLastStep, double removeFilterTime);
        static void
        handleRobots(robotMap &robots,
                     const google::protobuf::RepeatedPtrField <proto::SSL_DetectionRobot> &observations,
                     double filterGrabDistance, double timeCapture, uint cameraID);
    };
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
