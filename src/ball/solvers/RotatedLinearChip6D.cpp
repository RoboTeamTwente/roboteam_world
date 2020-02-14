//
// Created by rolf on 07-02-20.
//

#include "ball/solvers/RotatedLinearChip6D.h"
#include "BouncePoint.h"
void RotatedLinearChip6D::solve(std::vector<BallObservation> observations,
        const std::map<unsigned int, Camera> &cameras) {
    if (observations.size() < 3) {
        std::cerr << "Not enough points to create a trajectory" << std::endl;
        return;
    }

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(2*observations.size(), 6);
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(2*observations.size());

    double firstTime = observations[0].time;
    int i = 0;
    //HACK: For now assume only one camera for entire array
    auto cam = cameras.find(observations[0].cameraID);
    if (cam == cameras.end()) {
        std::cerr << "Could not find camera info, aborting!" << std::endl;
        return;
    }
    const auto &detectionCam = cam->second;
    Eigen::Quaterniond rotation=detectionCam.worldToCamRotation().normalized();
    Eigen::Vector3d camPos=detectionCam.worldPos();
    Eigen::Vector3d camGravity= rotation * Eigen::Vector3d(0,0,-9.81);
    double gx = camGravity(0);
    double gy = camGravity(1);
    double gz = camGravity(2);
    std::cout<<gx <<" "<<gy <<" "<< gz<<std::endl;
    for (const auto &observation : observations) {
        double time = observation.time - firstTime;
        Eigen::Vector3d obsPos(observation.ball.x()/1000.0,observation.ball.y()/1000.0,0.021333);
        Eigen::Vector3d obsPosCamFrame= rotation*(obsPos-camPos);
        double alpha = obsPosCamFrame.x()/obsPosCamFrame.z();
        double beta = obsPosCamFrame.y()/obsPosCamFrame.z();

        matrix(i*2, 0) = alpha;
        matrix(i*2, 1) = alpha*time;
        matrix(i*2, 2) = -1;
        matrix(i*2, 3) = -time;
        matrix(i*2, 4) = 0;
        matrix(i*2, 5) = 0;
        vector(i*2) = 0.5*time*time*(gx-alpha*gz);

        matrix(i*2 + 1, 0) = beta;
        matrix(i*2 + 1, 1) = beta*time;
        matrix(i*2 + 1, 2) = 0;
        matrix(i*2 + 1, 3) = 0;
        matrix(i*2 + 1, 4) = -1;
        matrix(i*2 + 1, 5) = -time;
        vector(i*2 + 1) = 0.5*time*time*(gy-beta*gz);
        i ++;
    }
    Eigen::VectorXd data = matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vector); //TODO look at alternative numerical methods for speed
    Eigen::Vector3d pos(data(2),data(4),data(0));
    Eigen::Vector3d vel(data(3),data(5),data(1));

    Eigen::Vector3d worldPos= rotation.inverse()*pos + camPos;
    Eigen::Vector3d worldVel= rotation.inverse()*vel;
    std::optional<BouncePoint> point=calculateBouncePoint(worldPos,worldVel,9.81,0.021333);
    if (point){
        std::cout << "bouncePos: " << (*point).bouncePos.x() <<" " << (*point).bouncePos.y() <<" time: " << (*point).bounceTime <<std::endl;
    }
    std::cout << "time: " << observations[observations.size()-1].time - firstTime << std::endl;
    std::cout << " z_0: " << worldPos(2);
    std::cout << " v_z: " << worldVel(2);
    std::cout << " x_0: " << worldPos(0);
    std::cout << " v_x: " << worldVel(0);
    std::cout << " y_0: " << worldPos(1);
    std::cout << " v_y: " << worldVel(1) << std::endl;

}
