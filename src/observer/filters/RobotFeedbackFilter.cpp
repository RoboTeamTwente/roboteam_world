//
// Created by rolf on 29-10-21.
//

#include "observer/filters/RobotFeedbackFilter.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/RobotShape.h"

void RobotFeedbackFilter::process(const proto::RobotData& data) {
    auto& map = data.isyellow() ? lastYellowFeedback : lastBlueFeedback;
    map.clear();
    for (const auto& feedback : data.receivedfeedback()) {
        map[feedback.id()] = feedback;
    }
}

std::vector<proto::RobotFeedback> RobotFeedbackFilter::getData(bool teamIsYellow) const {
    std::vector<proto::RobotFeedback> feedback;
    const auto& map = teamIsYellow ? lastYellowFeedback : lastBlueFeedback;
    feedback.reserve(map.size());
    for (const auto& elem : map) {
        feedback.emplace_back(elem.second);
    }
    return feedback;
}
std::optional<proto::RobotFeedback> RobotFeedbackFilter::getDataFilteredBallPos(bool teamIsYellow, const proto::WorldRobot& robot,
                                                                                const proto::Vector2f& ballPos) {
    const auto& map = teamIsYellow ? lastYellowFeedback : lastBlueFeedback;
    auto feedback = map.find(robot.id());
    if(feedback == map.end()){
        return std::nullopt;
    }
    proto::RobotFeedback copy = feedback->second;
    if(copy.hasball()){
        rtt::Vector2 robotFrontPos(robot.pos().x(),robot.pos().y());
        constexpr double centerToFrontLength = 0.07; //TODO: fix magic constant (make team dependant)
        //TODO: make estimation using robotBallPos?
        robotFrontPos+= rtt::Vector2(rtt::Angle(robot.angle()),centerToFrontLength);
        rtt::Vector2 ball(ballPos.x(),ballPos.y());

        constexpr double maxBallSensorDist = 0.08; //[m]

        if(ball.dist(robotFrontPos) >  maxBallSensorDist){
            copy.set_hasball(false);
            copy.set_ballpos(0.0);
        }
    }

    return copy;
}
