#ifndef WORLDHANDLER_H
#define WORLDHANDLER_H
#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/RobotFeedback.pb.h"
#include "roboteam_proto/Subscriber.h"
#include <roboteam_proto/Publisher.h>
#include <net/robocup_ssl_client.h>
#include "kalman/WorldFilter.h"
#include "util/KalmanFilter.h"

namespace world {

class WorldHandler {
 private:
  proto::Publisher<proto::World> *world_pub;
  proto::Publisher<proto::SSL_Referee> *ref_pub;
  proto::Publisher<proto::SSL_GeometryData> *geom_pub;
  proto::Subscriber<proto::RobotFeedback> *feedback_yellow_sub;
  proto::Subscriber<proto::RobotFeedback> *feedback_blue_sub;

  double lastPacketTime;
  WorldFilter *KF;
  RoboCupSSLClient *vision_client;
  RoboCupSSLClient *refbox_client;

 public:
  WorldHandler() = default;

  /*
   * Setup a world with a kalmanfilter, and initialize the publishers for publishing data.
   */
  void init();
  void start();
  void handleVisionPackets(proto::SSL_WrapperPacket &vision_packet);
  void handleRefboxPackets(proto::SSL_Referee &ref_packet) const;

  /**
   * Adds the received robot feedback of yellow to the WorldFilter.
   * @param feedback Received robot feedback of yellow
   */
  void handleFeedbackYellow(proto::RobotFeedback &feedback);
  /**
   * Adds the received robot feedback of blue to the WorldFilter.
   * @param feedback Received robot feedback of blue
   */
  void handleFeedbackBlue(proto::RobotFeedback &feedback);

  void setupSSLClients();
};

}
#endif