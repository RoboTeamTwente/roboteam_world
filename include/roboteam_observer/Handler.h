#ifndef WORLDHANDLER_H
#define WORLDHANDLER_H

#include <proto/State.pb.h>

#include <RobotFeedbackNetworker.hpp>
#include <WorldNetworker.hpp>

#include <observer/Observer.h>
#include <utility>
#include "RobocupReceiver.h"
#include <memory>
#include <vector>
#include <exception>
#include <fstream>

class Handler {
   private:
    std::unique_ptr<rtt::net::RobotFeedbackSubscriber> feedbackSubscriber;
    std::unique_ptr<rtt::net::WorldPublisher> worldPublisher;

    std::unique_ptr<RobocupReceiver<proto::SSL_WrapperPacket>> vision_client;
    std::unique_ptr<RobocupReceiver<proto::SSL_Referee>> referee_client;

    Observer observer;
    std::vector<rtt::RobotsFeedback> receivedRobotData;
    std::mutex sub_mutex;

    std::ofstream blueTeamLogger;
    std::ofstream yellowTeamLogger;

    void logWorldRobots(const proto::World&);
   public:
    explicit Handler(bool shouldLog);
    ~Handler();

    /*
     * Setup a world with a kalmanfilter, and initialize the publishers for publishing data.
     */
    bool initializeNetworkers();
    bool setupSSLClients();
    // Sets up log files. Returns success
    bool setupLogFiles();

    void start();
    std::vector<proto::SSL_WrapperPacket> receiveVisionPackets();
    std::vector<proto::SSL_Referee> receiveRefereePackets();
    void onRobotFeedback(const rtt::RobotsFeedback& feedback);
};

class FailedToInitializeNetworkersException : public std::exception {
    const char* what() const throw();
};
class FailedToSetupSSLClients : public std::exception {
    const char* what() const throw();
};


#endif