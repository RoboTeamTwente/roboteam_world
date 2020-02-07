//
// Created by rolf on 07-02-20.
//

#ifndef RTT_CAMERA_H
#define RTT_CAMERA_H

#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>
#include <Eigen/Dense>
class Camera {
    public:
        explicit Camera(const proto::SSL_GeometryCameraCalibration& protoCam);
        [[nodiscard]] Eigen::Vector2d fieldToImage(const Eigen::Vector3d& fieldPoint) const;
        Eigen::Vector3d imageToField(const Eigen::Vector2d& imagePoint, double assumedHeight) const;
    private:
        [[nodiscard]] Eigen::Vector2d radialDistortion(Eigen::Vector2d& imagePoint) const;
        [[nodiscard]] double radialDistortion(double radius) const;
        [[nodiscard]] double radialDistortionInv(double radius) const;
        [[nodiscard]] Eigen::Vector2d radialDistortionInv(Eigen::Vector2d& imagePoint) const;
        double rayPlaneIntersection(Eigen::Vector3d planeOrigin, Eigen::Vector3d planeNormal, Eigen::Vector3d rayOrigin,Eigen::Vector3d rayVector) const;
        double focalLength;
        Eigen::Vector2d principalPoint;
        Eigen::Vector3d position;
        Eigen::Vector3d translation;
        Eigen::Quaterniond orientation;
        double distortion;

};

#endif //RTT_CAMERA_H
