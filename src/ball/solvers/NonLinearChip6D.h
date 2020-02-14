//
// Created by rolf on 11-02-20.
//

#ifndef RTT_NONLINEARCHIP6D_H
#define RTT_NONLINEARCHIP6D_H
#include <nlopt.hpp>
#include <Eigen/Dense>
#include <data/Camera.h>
#include <data/BallObservation.h>
double valueFunction(const std::vector<double> &x, std::vector<double> &grad, void* f_data);

class NonLinearChip6D {
    public:
        NonLinearChip6D();
        void solve(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras);
        void runOptimization();
        struct Data{
          Eigen::MatrixXd matrix;
          Eigen::VectorXd vector;
        };
    private:
        bool setData(std::vector<BallObservation> observations, const std::map<unsigned int,Camera> &cameras);
        nlopt::opt algorithm;

        Data* data;
};

#endif //RTT_NONLINEARCHIP6D_H
