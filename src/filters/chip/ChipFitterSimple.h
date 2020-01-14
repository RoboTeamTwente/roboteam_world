//
// Created by rolf on 04-01-20.
//

#ifndef ROBOTEAM_WORLD_CHIPFITTERSIMPLE_H
#define ROBOTEAM_WORLD_CHIPFITTERSIMPLE_H


#include <vector>
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>
#include "filters/BallFilter.h"
#include <QVector3D>
#include <QQuaternion>

class ChipFitterSimple {
public:
    void reconstruct(std::vector<BallObservation> observations,const std::map<int,proto::SSL_GeometryCameraCalibration> &cameraCalibration);
    QVector3D toWorld(QVector3D camVec, QQuaternion camRot,QVector3D camPos);
    QVector3D toCamera(QVector3D worldVec,QQuaternion camRot, QVector3D camPos);
};


#endif //ROBOTEAM_WORLD_CHIPFITTERSIMPLE_H
