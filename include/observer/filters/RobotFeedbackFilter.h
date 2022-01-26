//
// Created by rolf on 29-10-21.
//

#ifndef RTT_ROBOTFEEDBACKFILTER_H
#define RTT_ROBOTFEEDBACKFILTER_H

#include <proto/RobotData.pb.h>
#include <proto/RobotFeedback.pb.h>
#include <proto/WorldRobot.pb.h>

#include <map>

class RobotFeedbackFilter {
   private:
    std::map<unsigned int, proto::RobotFeedback> lastBlueFeedback;
    std::map<unsigned int, proto::RobotFeedback> lastYellowFeedback;

   public:
    void process(const proto::RobotData& data);
    [[nodiscard]] std::vector<proto::RobotFeedback> getData(bool teamIsYellow) const;

    /**
     * Gets the robotfeedback of the given robot, but resets ballsensors data if the robot is not close to the ball
     * @return
     */
    [[nodiscard]] std::optional<proto::RobotFeedback> getDataFilteredBallPos(bool teamIsYellow, const proto::WorldRobot& robot,
                                                                             const proto::Vector2& ballPos);
};

#endif  // RTT_ROBOTFEEDBACKFILTER_H
