//
// Created by rolf on 11-02-20.
//

#include "NonLinearChip6D.h"
#include "BouncePoint.h"
NonLinearChip6D::NonLinearChip6D() {
    algorithm = nlopt::opt(nlopt::GN_CRS2_LM, 6);
}
void NonLinearChip6D::solve(std::vector<BallObservation> observations,
        const std::map<unsigned int, Camera> &cameras) {
    setData(observations, cameras);
    runOptimization();
}

double valueFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
    Eigen::VectorXd data = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < x.size(); ++ i) {
        data(i) = x[i];
    }
    NonLinearChip6D::Data* d = (NonLinearChip6D::Data*) f_data;
    Eigen::VectorXd result = d->matrix*data - d->vector;
    double sum = 0;
    for (int j = 0; j < result.size(); ++ j) {
        if (j != 2) {
            sum += abs(result(j));
        }else{
            sum += abs(result(j));
        }

    }
    return sum;
}
bool NonLinearChip6D::setData(std::vector<BallObservation> observations,
        const std::map<unsigned int, Camera> &cameras) {
    if (observations.size() < 3) {
        std::cerr << "Not enough points to create a trajectory" << std::endl;
        return false;
    }
    data = new Data(); //TODO: fix memory leak
    data->matrix = Eigen::MatrixXd::Zero(2*observations.size(), 6);
    data->vector = Eigen::VectorXd::Zero(2*observations.size());

    double firstTime = observations[0].time;
    const double gravity = 9.81;
    int i = 0;
    for (const auto &observation : observations) {
        auto cam = cameras.find(observation.cameraID);
        if (cam == cameras.end()) {
            std::cerr << "Could not find camera info, aborting!" << std::endl;
            return false;
        }
        const auto &detectionCam = cam->second;
        Eigen::Vector3d camPos = detectionCam.worldPos();
        double time = observation.time - firstTime;
        double x = observation.ball.x()/1000.0;
        double y = observation.ball.y()/1000.0;
        double alpha = (x - camPos.x())/(camPos.z() - 0.021333);
        double beta = (y - camPos.y())/(camPos.z() - 0.021333);

        data->matrix(i*2, 0) = alpha;
        data->matrix(i*2, 1) = alpha*time;
        data->matrix(i*2, 2) = 1;
        data->matrix(i*2, 3) = time;
        data->matrix(i*2, 4) = 0;
        data->matrix(i*2, 5) = 0;
        data->vector(i*2) = 0.5*gravity*time*time*alpha + x;

        data->matrix(i*2 + 1, 0) = beta;
        data->matrix(i*2 + 1, 1) = beta*time;
        data->matrix(i*2 + 1, 2) = 0;
        data->matrix(i*2 + 1, 3) = 0;
        data->matrix(i*2 + 1, 4) = 1;
        data->matrix(i*2 + 1, 5) = time;
        data->vector(i*2 + 1) = 0.5*gravity*time*time*beta + y;
        i ++;
    }
    return true;
}
void NonLinearChip6D::runOptimization() {
    algorithm.set_min_objective(valueFunction, data);
    std::vector<double> lowerBounds = {- 0.05, 0.2, - 3.1, - 4, - 0.1, - 4};
    std::vector<double> upperBounds = {0.3, 6, - 2.9, 4, 0.1, 4};
    algorithm.set_lower_bounds(lowerBounds);
    algorithm.set_upper_bounds(upperBounds);
    algorithm.set_maxtime(1.0);
    std::vector<double> initialGuess = {0.02133, 3.9, - 2.99, 0.46, - 0.00972, - 1.68};
    std::vector<double> x = initialGuess;
    double value;
    try {
        nlopt::result result = algorithm.optimize(x, value);
        std::optional<BouncePoint> point = calculateBouncePoint(Eigen::Vector3d(x[2], x[4], x[0]),
                Eigen::Vector3d(x[3], x[5], x[1]), 9.81, 0.021333);
        if (point) {
            std::cout << "bouncePos: " << (*point).bouncePos.x() << " " << (*point).bouncePos.y() << " time: "
                      << (*point).bounceTime << std::endl;
        }
        std::cout << " z_0: " << x[0];
        std::cout << " v_z: " << x[1];
        std::cout << " x_0: " << x[2];
        std::cout << " v_x: " << x[3];
        std::cout << " y_0: " << x[4];
        std::cout << " v_y: " << x[5] << std::endl;
    }
    catch (std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
}
