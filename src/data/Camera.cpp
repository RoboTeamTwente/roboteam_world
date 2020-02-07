//
// Created by rolf on 07-02-20.
//

#include "data/Camera.h"
Camera::Camera(const proto::SSL_GeometryCameraCalibration &protoCam)
        :
        position{Eigen::Vector3d(protoCam.derived_camera_world_tx(), protoCam.derived_camera_world_ty(),
                protoCam.derived_camera_world_tz())},
        translation{Eigen::Vector3d(protoCam.tx(), protoCam.ty(), protoCam.tz())},
        orientation{Eigen::Quaterniond(protoCam.q3(), protoCam.q0(), protoCam.q1(),
                protoCam.q2()).normalized()}, //We normalize here so we don't need to do it in calcuations repeatedly
        distortion{protoCam.distortion()},
        focalLength(protoCam.focal_length()),
        principalPoint{Eigen::Vector2d(protoCam.principal_point_x(), protoCam.principal_point_y())} {

}
Eigen::Vector2d Camera::fieldToImage(const Eigen::Vector3d& fieldPoint) const {
    //First transform the point into camera coordinates
    Eigen::Vector3d camCoorPoint = orientation*fieldPoint + translation;
    //Then project it onto the image:
    Eigen::Vector2d imageProjection(camCoorPoint.x()/camCoorPoint.z(), camCoorPoint.y()/camCoorPoint.z());
    //distort the point accordingly
    Eigen::Vector2d distortedProjection = radialDistortion(imageProjection);
    //Add the image transformation by scaling and translating the final image point.
    return focalLength*distortedProjection + principalPoint;
}
Eigen::Vector3d Camera::imageToField(const Eigen::Vector2d& imagePoint, double assumedHeight) const {
    //Translate imagePoint to coordinatesystem to undistort (with distortion centre at 0)
    Eigen::Vector2d translatedImagePoint = (imagePoint - principalPoint)/focalLength;
    //Undistort the point
    Eigen::Vector2d undistortedPoint = radialDistortionInv(translatedImagePoint);

    //Now we create a 3d ray on the z-axis
    Eigen::Vector3d ray(undistortedPoint.x(),undistortedPoint.y(),1.0);
    //We compute the transformation from camera to field
    Eigen::Quaterniond fieldToCamInverse = orientation.inverse();
    Eigen::Vector3d rayInCam = fieldToCamInverse * ray;
    Eigen::Vector3d zeroInCam= fieldToCamInverse * (-translation);
    rayInCam.normalize();//We need to normalize for the below calculation
    //Now compute the point where the ray intersects the field and return this point
    double t=rayPlaneIntersection(Eigen::Vector3d(0,0,assumedHeight),Eigen::Vector3d(0,0,1),
            zeroInCam,rayInCam);
    return zeroInCam+rayInCam*t;
}
double Camera::radialDistortion(double radius) const {
    double rd = 0;
    double a = distortion;
    double b = - 9.0*a*a*radius + a*sqrt(a*(12.0 + 81.0*a*radius*radius));
    b = (b < 0.0) ? (- pow(b, 1.0/3.0)) : pow(b, 1.0/3.0);
    rd = pow(2.0/3.0, 1.0/3.0)/b -
            b/(pow(2.0*3.0*3.0, 1.0/3.0)*a);
    return rd;
}

double Camera::radialDistortionInv(double radius) const {
    return radius*(1.0 + radius*radius*distortion);
}
Eigen::Vector2d Camera::radialDistortion(Eigen::Vector2d &imagePoint) const {
    double rd = radialDistortion(imagePoint.norm());
    imagePoint.normalize(); //We do in place normalization for speed
    Eigen::Vector2d distortedPoint = imagePoint*rd;
    return distortedPoint;
}
Eigen::Vector2d Camera::radialDistortionInv(Eigen::Vector2d &imagePoint) const {
    double rd = radialDistortionInv(imagePoint.norm());
    imagePoint.normalize();
    Eigen::Vector2d distortedPoint = imagePoint*rd;
    return distortedPoint;
}

double Camera::rayPlaneIntersection(Eigen::Vector3d planeOrigin, Eigen::Vector3d planeNormal, Eigen::Vector3d rayOrigin,
        Eigen::Vector3d rayVector) const {
    return (-planeNormal).dot(rayOrigin-planeOrigin) / (planeNormal.dot(rayVector));
}

