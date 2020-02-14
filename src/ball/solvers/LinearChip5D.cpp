//
// Created by rolf on 08-02-20.
//

#include "ball/solvers/LinearChip5D.h"
#include "BouncePoint.h"
#include <Eigen/Dense>
double LinearChip5D::solve(std::vector<BallObservation> observations, const std::map<unsigned int, Camera> &cameras, double timeOffset,bool print) {
    if (observations.size() < 3) {
        std::cerr << "Not enough points to create a trajectory" << std::endl;
        return std::numeric_limits<double>::max();
    }

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(2*observations.size(), 5);
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(2*observations.size());

    double firstTime = observations[0].time;
    const double gravity = 9.81;
    const double ballRadius = 0.021333;
    int i = 0;
    for (const auto &observation : observations) {
        auto cam = cameras.find(observation.cameraID);
        if (cam == cameras.end()) {
            std::cerr << "Could not find camera info, aborting!" << std::endl;
            return std::numeric_limits<double>::max();
        }
        const auto &detectionCam = cam->second;
        Eigen::Vector3d camPos = detectionCam.worldPos();
        double time = observation.time - firstTime + timeOffset;
        double x = observation.ball.x()/1000.0;
        double y = observation.ball.y()/1000.0;
        double alpha = (x - camPos.x())/(camPos.z() - ballRadius);
        double beta = (y - camPos.y())/(camPos.z() - ballRadius);

        //TODO: possible speed optimization using EIGEN_NO_DEBUG for bounds checking
        matrix(i*2, 0) = alpha*time;
        matrix(i*2, 1) = 1;
        matrix(i*2, 2) = time;
        matrix(i*2, 3) = 0;
        matrix(i*2, 4) = 0;
        vector(i*2) = 0.5*gravity*time*time*alpha + x - alpha*ballRadius;

        matrix(i*2 + 1, 0) = beta*time;
        matrix(i*2 + 1, 1) = 0;
        matrix(i*2 + 1, 2) = 0;
        matrix(i*2 + 1, 3) = 1;
        matrix(i*2 + 1, 4) = time;
        vector(i*2 + 1) = 0.5*gravity*time*time*beta + y - beta*ballRadius;
        i ++;
    }
    Eigen::VectorXd data = matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vector);
    Eigen::VectorXd res=matrix*data-vector;
    double l1norm=0;
    for (int j = 0; j < 2*observations.size(); ++ j) {
        l1norm+=abs(res(j));
    }
    if(print){
        std::optional<BouncePoint> point=calculateBouncePoint(Eigen::Vector3d(data(1),data(3),ballRadius),Eigen::Vector3d(data(2),data(4),data(0)),9.81,ballRadius);
        if (point){
            std::cout << "bouncePos: " << (*point).bouncePos.x() <<" " << (*point).bouncePos.y() <<" time: " << (*point).bounceTime +timeOffset<<std::endl;
        }
        std::cout << "time: " << observations[observations.size()-1].time - firstTime << "timeOffset: " <<timeOffset<<std::endl;
        std::cout << " z_0: " << ballRadius;
        std::cout << " v_z: " << data(0);
        std::cout << " x_0: " << data(1);
        std::cout << " v_x: " << data(2);
        std::cout << " y_0: " << data(3);
        std::cout << " v_y: " << data(4) << std::endl;
    }
    return l1norm;
}
void LinearChip5D::binSearch(std::vector<BallObservation> observations, const std::map<unsigned int, Camera> &cameras) {
    double timeOffset=0.025;

    double increment = timeOffset; //We search inbetween -0.025 to 0.075
    while(increment >1e-4){
        double resultLeft = solve(observations,cameras,timeOffset-1e-5);
        double resultRight = solve(observations,cameras,timeOffset+1e-5);
        if(resultLeft>resultRight){
            timeOffset+=increment;
        }
        else{
            timeOffset-=increment;
        }
        increment/=2;
    }
    solve(observations,cameras,timeOffset,true);
}
