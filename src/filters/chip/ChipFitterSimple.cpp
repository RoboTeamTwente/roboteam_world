//
// Created by rolf on 04-01-20.
//

#include "ChipFitterSimple.h"
#include "iostream"


void ChipFitterSimple::reconstruct(std::vector<BallObservation> observations, const std::map<int,proto::SSL_GeometryCameraCalibration> &cameraCalibration) {
    //assume observations are sorted by time.
    int obsCount=observations.size();
    if (obsCount<150){ //not enough equations to determine all variables.
        return;
    }
    arma::dmat obsMatrix(obsCount*2,3);
    arma::dvec obsVector(obsCount*2);
    double firstTime=observations[0].time;
    if (cameraCalibration.find(observations[0].cameraID)==cameraCalibration.end()){
        std::cout<<"Could not find appropriate camera!"<<std::endl;
        return;
    }

    const proto::SSL_GeometryCameraCalibration &cam=cameraCalibration.at(observations[0].cameraID);
    QQuaternion rotation(cam.q3(),cam.q0(),cam.q1(),cam.q2());
    QVector3D position(cam.tx()/1000.0,cam.ty()/1000.0,cam.tz()/1000.0);
    std::cout<<"observations: "<<obsCount<<std::endl;
    std::cout<<observations[obsCount-1].time<<std::endl;
    double kickX=-5.0;
    double kickY=0.0;
    for (int i = 0; i < obsCount; i++) {
        if (observations[i].cameraID!=0){
            std::cerr<<"NOT THE RIGHT CAM"<<std::endl;
            return;
        }
        QVector3D ballPos(observations[i].ball.x(),observations[i].ball.y(),23);
        double x= ballPos.x()/1000.0;
        double y= ballPos.y()/1000.0;

        double t=observations[i].time-firstTime;
        double gz = 9.81;

        obsMatrix(2*i,0)=position.z()*t;
        obsMatrix(2*i,1)=0;
        obsMatrix(2*i,2)=(x-position.x())*t;

        obsVector(2*i)=0.5*gz*t*t*(x-position.x()) + x*position.z() - kickX*position.z();

        obsMatrix(2*i+1,0)=0;
        obsMatrix(2*i+1,1)=position.z()*t;
        obsMatrix(2*i+1,2)=(y-position.y())*t;

        obsVector(2*i+1)=0.5*gz*t*t*(y-position.y()) + y*position.z()-kickY*position.z();
    }
    arma::dvec sol=arma::solve(obsMatrix,obsVector);

    std::cout<<sol<<std::endl;



}

QVector3D ChipFitterSimple::toCamera(QVector3D worldVec, QQuaternion camRot, QVector3D camPos) {
    return camRot.rotatedVector(worldVec-camPos);
}
QVector3D ChipFitterSimple::toWorld(QVector3D camVec, QQuaternion camRot, QVector3D camPos) {
    return (-camRot).rotatedVector(camVec)+camPos;
}