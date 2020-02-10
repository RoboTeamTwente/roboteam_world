//
// Created by rolf on 07-02-20.
//

#include "ball/solvers/LinearChip6D.h"
#include <Eigen/Dense>
void LinearChip6D::solve(std::vector<BallObservation> observations, const std::map<unsigned int, Camera> &cameras) {
    if (observations.size() < 3) {
        std::cerr << "Not enough points to create a trajectory" << std::endl;
        return;
    }

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(2*observations.size(), 6);
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(2*observations.size());

    double firstTime = observations[0].time;
    const double gravity = 9.81;
    int i = 0;
    for (const auto &observation : observations) {
        auto cam = cameras.find(observation.cameraID);
        if (cam == cameras.end()) {
            std::cerr << "Could not find camera info, aborting!" << std::endl;
            return;
        }
        const auto &detectionCam = cam->second;
        Eigen::Vector3d camPos = detectionCam.worldPos();
        double time = observation.time - firstTime;
        double x = observation.ball.x()/1000.0;
        double y = observation.ball.y()/1000.0;
        double alpha = (x - camPos.x())/(camPos.z()-0.021333);
        double beta = (y - camPos.y())/(camPos.z()-0.021333);

        matrix(i*2, 0) = alpha;
        matrix(i*2, 1) = alpha*time;
        matrix(i*2, 2) = 1;
        matrix(i*2, 3) = time;
        matrix(i*2, 4) = 0;
        matrix(i*2, 5) = 0;
        vector(i*2) = 0.5*gravity*time*time*alpha + x;

        matrix(i*2 + 1, 0) = beta;
        matrix(i*2 + 1, 1) = beta*time;
        matrix(i*2 + 1, 2) = 0;
        matrix(i*2 + 1, 3) = 0;
        matrix(i*2 + 1, 4) = 1;
        matrix(i*2 + 1, 5) = time;
        vector(i*2 + 1) = 0.5*gravity*time*time*beta + y;
        i ++;
    }
    Eigen::VectorXd data = matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vector);
    std::cout << "time: " << observations[observations.size()-1].time - firstTime << std::endl;
    std::cout << " z_0: " << data(0);
    std::cout << " v_z: " << data(1);
    std::cout << " x_0: " << data(2);
    std::cout << " v_x: " << data(3);
    std::cout << " y_0: " << data(4);
    std::cout << " v_y: " << data(5) << std::endl;
//    data = matrix.colPivHouseholderQr().solve(vector);
//    std::cout << "time: " << observations[8].time - firstTime << std::endl;
//    std::cout << " z_0: " << data(0);
//    std::cout << " v_z: " << data(1);
//    std::cout << " x_0: " << data(2);
//    std::cout << " v_x: " << data(3);
//    std::cout << " y_0: " << data(4);
//    std::cout << " v_y: " << data(5) << std::endl;
//    data = (matrix.transpose()*matrix).ldlt().solve(matrix.transpose()*vector);
//    std::cout << "time: " << observations[8].time - firstTime << std::endl;
//    std::cout << " z_0: " << data(0);
//    std::cout << " v_z: " << data(1);
//    std::cout << " x_0: " << data(2);
//    std::cout << " v_x: " << data(3);
//    std::cout << " y_0: " << data(4);
//    std::cout << " v_y: " << data(5) << std::endl;
//
//    data = matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vector);
//    std::cout << "time: " << observations[8].time - firstTime << std::endl;
//    std::cout << " z_0: " << data(0);
//    std::cout << " v_z: " << data(1);
//    std::cout << " x_0: " << data(2);
//    std::cout << " v_x: " << data(3);
//    std::cout << " y_0: " << data(4);
//    std::cout << " v_y: " << data(5) << std::endl;

}
