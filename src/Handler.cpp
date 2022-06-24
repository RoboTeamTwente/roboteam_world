#include "Handler.h"

#include <roboteam_utils/Time.h>
#include <roboteam_utils/Timer.h>
#include <sstream>
#include <algorithm>
#include <proto/messages_robocup_ssl_wrapper.pb.h>

constexpr char MARPLE_DELIMITER = ',';
constexpr char ROBOT_LOG_MARPLE_HEADER[] = "time,id,posX,posY,angle,velX,velY,angleVel,hasBall,ballPos,sensorWorks,xSensCalibrated,capCharged,battery,signal";

Handler::Handler(bool shouldLog) {
    if (shouldLog) {
        if (!this->setupLogFiles())
            throw std::runtime_error("Failed to open file(s) for logging");
    }
}
Handler::~Handler() {
    this->blueTeamLogger.close();
    this->yellowTeamLogger.close();
}

void Handler::start() {
    if (!initializeNetworkers()) {
        throw FailedToInitializeNetworkersException();
    }
    if (!this->setupSSLClients()) {
        throw FailedToSetupSSLClients();
    }

    roboteam_utils::Timer t;

    t.loop(
        [&]() {
            auto vision_packets = receiveVisionPackets();
            auto referee_packets = receiveRefereePackets();
            std::vector<rtt::RobotsFeedback> robothub_info;
            {
                std::lock_guard guard(sub_mutex);
                std::swap(robothub_info, this->receivedRobotData);
            }

            auto state = observer.process(vision_packets,referee_packets,robothub_info); //TODO: fix time extrapolation

            t.limit([&]() {
                this->logWorldRobots(state.last_seen_world());
            },
            60);

            std::size_t iterations = 0;
            bool sent = false;
            while(iterations < 10){
              auto bytesSent = worldPublisher->publish(state);
              if(bytesSent > 0){
                sent = true;
                break;
              }
              iterations++;
            }
            if(!sent){
              std::cout<<"could not send data on publisher!"<<std::endl;
            }
        },
        100);
}
bool Handler::initializeNetworkers() {
    this->worldPublisher = std::make_unique<rtt::net::WorldPublisher>();

    this->feedbackSubscriber = std::make_unique<rtt::net::RobotFeedbackSubscriber>([&](const rtt::RobotsFeedback& feedback) {
        onRobotFeedback(feedback);
    });

    return this->worldPublisher != nullptr && this->feedbackSubscriber != nullptr;
}

bool Handler::setupSSLClients() {
    bool success = true;
    constexpr quint16 DEFAULT_VISION_PORT = 10006;
    constexpr quint16 DEFAULT_REFEREE_PORT = 10003;

    const QString SSL_VISION_SOURCE_IP = "224.5.23.2";
    const QString SSL_REFEREE_SOURCE_IP = "224.5.23.1";
    
    this->vision_client = std::make_unique<RobocupReceiver<proto::SSL_WrapperPacket>>(QHostAddress(SSL_VISION_SOURCE_IP),DEFAULT_VISION_PORT);
    this->referee_client = std::make_unique<RobocupReceiver<proto::SSL_Referee>>(QHostAddress(SSL_REFEREE_SOURCE_IP),DEFAULT_REFEREE_PORT);

    success = vision_client != nullptr && referee_client != nullptr;
    std::cout << "Vision  : " << SSL_VISION_SOURCE_IP.toStdString() << ":" << DEFAULT_VISION_PORT << std::endl;
    std::cout << "Referee  : " << SSL_REFEREE_SOURCE_IP.toStdString() << ":" << DEFAULT_REFEREE_PORT << std::endl;

    success &= vision_client->connect();
    success &= referee_client->connect();
    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    return success;
}

bool Handler::setupLogFiles() {
    auto time = Time::getDate('-') + "_" + Time::getTime('-');
    auto suffix = "_MARPLE.csv";

    this->blueTeamLogger.open("BLUE_TEAM_" + time + suffix);
    this->yellowTeamLogger.open("YELLOW_TEAM_" + time + suffix);

    if (!this->blueTeamLogger.is_open() || !this->yellowTeamLogger.is_open()) {
        return false;
    }

    // Add headers to explain values and add std::fixed to floating points are not in scientific notation
    this->blueTeamLogger << ROBOT_LOG_MARPLE_HEADER << std::fixed << std::endl;
    this->yellowTeamLogger << ROBOT_LOG_MARPLE_HEADER << std::fixed << std::endl;
    return true;
}

void writeRobotToStream(const proto::WorldRobot& bot, long time, std::ofstream& stream) {
    // time,id,posX,posY,angle,velX,velY,angleVel,hasBall,ballPos,sensorWorks,xSensCalibrated,capCharged,battery,signal
    stream
        << time << MARPLE_DELIMITER
        << bot.id() << MARPLE_DELIMITER
        << bot.pos().x() << MARPLE_DELIMITER
        << bot.pos().y() << MARPLE_DELIMITER
        << bot.angle() << MARPLE_DELIMITER
        << bot.vel().x() << MARPLE_DELIMITER
        << bot.vel().y() << MARPLE_DELIMITER
        << bot.w() << MARPLE_DELIMITER
        << bot.feedbackinfo().has_ball() << MARPLE_DELIMITER
        << bot.feedbackinfo().ball_position() << MARPLE_DELIMITER
        << bot.feedbackinfo().ball_sensor_is_working() << MARPLE_DELIMITER
        << bot.feedbackinfo().xsens_is_calibrated() << MARPLE_DELIMITER
        << bot.feedbackinfo().capacitor_is_charged() << MARPLE_DELIMITER
        << bot.feedbackinfo().battery_level() << MARPLE_DELIMITER
        << bot.feedbackinfo().signal_strength() << std::endl;
}

void Handler::logWorldRobots(const proto::World& world) {
    if (!this->blueTeamLogger.is_open() || !this->yellowTeamLogger.is_open()) return;

    auto now = std::chrono::system_clock::now().time_since_epoch().count();

    for (const auto& robot : world.blue()) {
        writeRobotToStream(robot, now, this->blueTeamLogger);
    }
    for (const auto& robot : world.yellow()) {
        writeRobotToStream(robot, now, this->yellowTeamLogger);
    }
}

std::vector<proto::SSL_WrapperPacket> Handler::receiveVisionPackets() {
    std::vector<proto::SSL_WrapperPacket> receivedPackets;
    bool ok = vision_client->receive(receivedPackets);
    if(!ok){
      std::cout<<"error receiving vision messages"<<std::endl;
    }
    return receivedPackets;
}
std::vector<proto::SSL_Referee> Handler::receiveRefereePackets()  {
  std::vector<proto::SSL_Referee> receivedPackets;
  bool ok = referee_client->receive(receivedPackets);
  if(!ok){
    std::cout<<"error receiving referee messages"<<std::endl;
  }
  return receivedPackets;
}

void Handler::onRobotFeedback(const rtt::RobotsFeedback& feedback) {
    std::lock_guard guard(sub_mutex);
    receivedRobotData.push_back(feedback);
}

const char* FailedToInitializeNetworkersException::what() const noexcept(true) { return "Failed to initialize networker(s). Is another observer running?"; }
const char* FailedToSetupSSLClients::what() const noexcept(true){ return "Failed to setup SSL client(s). Is another observer running?"; }