#ifndef WIFI_ESTIMATOR_H
#define WIFI_ESTIMATOR_H

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <vector>
using namespace std;

#include "parameter.h"

#include <ros/console.h>

class SolutionContainer
{
public:
    Vector3d p, v, g;
    Quaterniond q;
    Vector3d p_ap[NUMBER_OF_AP];
};

class WiFiEstimator
{
public:
    WiFiEstimator();

    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    SolutionContainer processWiFi(const vector<pair<int, Vector3d>> &wifi);

private:
    const Matrix3d acc_cov = 1e-2 * Matrix3d::Identity();
    const Matrix3d gra_cov = 1e-6 * Matrix3d::Identity();
    const Matrix3d gyr_cov = 1e-4 * Matrix3d::Identity();

    int frame_count;
    double current_time;

    MatrixXd A;
    VectorXd b;

    VectorXd x;
    Vector3d g;

    Matrix3d             Rs[WINDOW_SIZE + 1];

    Matrix<double, 7, 1> IMU_linear[WINDOW_SIZE + 1];
    Matrix3d             IMU_angular[WINDOW_SIZE + 1];
    vector<Vector3d>     wifi_measurement[WINDOW_SIZE + 1];

    Matrix<double, 6, 6> IMU_cov[WINDOW_SIZE + 1];
    Matrix<double, 9, 9> IMU_cov_nl[WINDOW_SIZE + 1];

    Matrix<double, 9, 1> odometry[WINDOW_SIZE + 1];

    SolutionContainer solveOdometry();
    void solveOdometryLinear();
    void marginalize();
};



#endif
