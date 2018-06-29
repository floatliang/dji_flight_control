#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/console.h>

#define SIMULATION 1 //simulation or experiment

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q);

class WifiEkf
{
public:
    WifiEkf();
    void init(double t, const Eigen::Vector3d &_g);
    void predict(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void update(double z);
    void update(Eigen::VectorXd zz);
    double measurement(const Eigen::Vector3d &_p, const Eigen::Matrix3d &_q, const Eigen::Vector3d &_ap);
    Eigen::Matrix<double, 1, 12> jacobian();
    Eigen::Matrix<double, 1, 12> jacobian(int ap_id);
    Eigen::Matrix<double, 1, 12> myJacobian(const Eigen::Vector3d &_delta_p, const Eigen::Vector3d &_delta_theta, const Eigen::Vector3d &_delta_ap);
    
    Eigen::Matrix<double, 1, 12> myJacobian(const Eigen::Vector3d &_delta_p, const Eigen::Vector3d &_delta_theta, const Eigen::Vector3d &_delta_ap, int ap_id);

    Eigen::Vector3d g; //gravity

    double cur_t;
    Eigen::Vector3d p, v; //position, velocity
    Eigen::Matrix3d q; //rotation
    Eigen::Vector3d ap; // ap
    std::vector<Eigen::Vector3d> APs;

    Eigen::Vector3d delta_p; // ap
    Eigen::Vector3d delta_ap; // ap
    Eigen::Vector3d delta_theta; // ap

    Eigen::Matrix<double, 12, 12> P; //cov of about state

    const Eigen::Matrix3d acc_cov = 0.1 * 0.01 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d gyr_cov = 0.01 * 0.001 * Eigen::Matrix3d::Identity();
    const double W_cov = 1.0;
};
