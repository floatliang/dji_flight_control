#include "wifi_ekf.h"

#define EPS 	1e-6
#define R  		0.06
#define LANDA 	0.05168

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << 0, -q(2), q(1),
        q(2), 0, -q(0),
        -q(1), q(0), 0;
    return ans;
}

WifiEkf::WifiEkf():
    g(Eigen::Vector3d::Zero()),
    p(Eigen::Vector3d::Zero()),
    v(Eigen::Vector3d::Zero()),
    q(Eigen::Matrix3d::Identity()),
    ap(Eigen::Vector3d::Zero()),
	
    delta_p(Eigen::Vector3d::Zero()),
    delta_ap(Eigen::Vector3d::Zero()),
    delta_theta(Eigen::Vector3d::Zero()),
    
    P(Eigen::Matrix<double, 12, 12>::Zero())
{
}

void WifiEkf::init(double t, const Eigen::Vector3d &_g)
{
    cur_t = t;
    ROS_INFO_STREAM("init filter with g body: " << g.transpose());
    q = Eigen::Quaterniond::FromTwoVectors(_g, Eigen::Vector3d(0.0, 0.0, -1.0)).toRotationMatrix(); // R^w_b
    g = q * _g;
    ROS_INFO_STREAM("aligned g world: " << g.transpose());
    p = Eigen::Vector3d(2, 0, 0);
    //p = Eigen::Vector3d(10, 10, 10);
    v.setZero();
    ap = Eigen::Vector3d(0, 0, 0);
    //ap = Eigen::Vector3d(10, -10, 10);
    P.block<3, 3>(9, 9) = 0.001 * Eigen::Matrix3d::Identity();
}

void WifiEkf::predict(double t, const Eigen::Vector3d &linear_acceleration_body, const Eigen::Vector3d &angular_velocity_body)
{
    double dt = t - cur_t;
    cur_t = t;
    Eigen::Quaterniond dq(1,
                          angular_velocity_body(0) * dt / 2,
                          angular_velocity_body(1) * dt / 2,
                          angular_velocity_body(2) * dt / 2);
    dq.w() = sqrt(1 - dq.vec().transpose() * dq.vec() );
    q = (Eigen::Quaterniond(q) * dq).normalized().toRotationMatrix();

	delta_theta(0) = angular_velocity_body(0)*dt;
	delta_theta(1) = angular_velocity_body(1)*dt;
	delta_theta(2) = angular_velocity_body(2)*dt;

    Eigen::Vector3d linear_acceleration_world = q * linear_acceleration_body - g;
    v += linear_acceleration_world * dt;
    delta_p = v * dt + 0.5 * linear_acceleration_world * dt * dt;
    p += v * dt + 0.5 * linear_acceleration_world * dt * dt;


    {
        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Zero();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 6) = -q * skewSymmetric(linear_acceleration_body);
        F.block<3, 3>(6, 6) = -skewSymmetric(angular_velocity_body);

        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        Q.block<3, 3>(0, 0) = acc_cov;
        Q.block<3, 3>(3, 3) = gyr_cov;

        Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();
        G.block<3, 3>(3, 0) = -q;
        G.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();

        P.block<9, 9>(0, 0) = (Eigen::Matrix<double, 9, 9>::Identity() + dt * F) * P.block<9, 9>(0, 0) * (Eigen::Matrix<double, 9, 9>::Identity() + dt * F).transpose() + (dt * G) * Q * (dt * G).transpose();
    }
}

void WifiEkf::update(double z)
{
    ROS_INFO_STREAM("P1: " << P.diagonal().transpose());
    double y = z - measurement(p, q, ap);
    printf("%f %f\n", z, measurement(p, q, ap));
    //Eigen::Matrix<double, 1, 12> H = jacobian();
	//std::cout << "H=\n" << H << std::endl;
	Eigen::Matrix<double, 1, 12> Hp = myJacobian(delta_p, delta_theta, delta_ap);
	//std::cout << "Hp=\n" << Hp << std::endl;
    double s = Hp * P * Hp.transpose() + W_cov;
    Eigen::Matrix<double, 12, 1> K = P * Hp.transpose() * (1.0 / s);
    P = (Eigen::Matrix<double, 12, 12>::Identity() - K * Hp) * P;
    Eigen::Matrix<double, 12, 1> dx = K * y;
    p += dx.segment<3>(0);
    v += dx.segment<3>(3);
    q = (Eigen::Quaterniond(q) * Eigen::Quaterniond(1.0, dx(6) / 2, dx(7) / 2, dx(8) / 2)).normalized().toRotationMatrix();
    ap += dx.segment<3>(9);
	delta_ap = dx.segment<3>(9);
    ROS_INFO_STREAM("P2: " << P.diagonal().transpose());
}


double WifiEkf::measurement(const Eigen::Vector3d &_p, const Eigen::Matrix3d &_q, const Eigen::Vector3d &_ap)
{
    Eigen::Vector3d distance = (_ap - _p).normalized();
    Eigen::Vector3d antenna = _q.col(0);
    return distance.dot(antenna);
}

Eigen::Matrix<double, 1, 12> WifiEkf::myJacobian(const Eigen::Vector3d &_delta_p, const Eigen::Vector3d &_delta_theta, const Eigen::Vector3d &_delta_ap)
{
    Eigen::Matrix<double, 1, 12> J = Eigen::Matrix<double, 1, 12>::Zero();
	//double C = -2*M_PI*R/LANDA;
	double C = -1;
	double N = (p(0) - ap(0) ) * (q(0,0) + _delta_theta(2)*q(0,1) - _delta_theta(1)*q(0,2) ) + (p(1) - ap(1)) * (q(1,0) + _delta_theta(2)*q(1,1) - _delta_theta(1)*q(1,2) ) + (p(2) - ap(2)) * (q(2,0) + _delta_theta(2)*q(2,1) - _delta_theta(1)*q(2,2) );
	double M = (p - ap).norm();
	for(int i = 0; i < 3; ++i)
	{
		J(i) = (C*M*(q(i,0) + _delta_theta(2)*q(i,1) - _delta_theta(1)*q(i,2) ) - (p(i) - ap(i))*C*N*(1.0/M) )/(M*M);
		J(i+3) = 0;
		J(i+9) = (-1*C*M*(q(i,0) + _delta_theta(2)*q(i,1) - _delta_theta(1)*q(i,2) ) + (p(i) - ap(i))*C*N*(1.0/M) )/(M*M);
	}
	J(6) = 0;
	J(7) = -1*C/M * (q(0,2)*(p(0)-ap(0)) + q(1,2)*(p(1)-ap(1)) + q(2,2)*(p(2)-ap(2)) );
	J(8) = C/M * (q(0,1)*(p(0)-ap(0)) + q(1,1)*(p(1)-ap(1)) + q(2,1)*(p(2)-ap(2)) );
	return J;	
}

Eigen::Matrix<double, 1, 12> WifiEkf::jacobian()
{
    Eigen::Vector3d eps_v[3];
    eps_v[0] = Eigen::Vector3d(EPS, 0.0, 0.0);
    eps_v[1] = Eigen::Vector3d(0.0, EPS, 0.0);
    eps_v[2] = Eigen::Vector3d(0.0, 0.0, EPS);

    Eigen::Matrix<double, 1, 12> J = Eigen::Matrix<double, 1, 12>::Zero();
    for (int i = 0; i < 3; i++)
    {
        J(0, 0 + i) = measurement(p + eps_v[i], q, ap) - measurement(p - eps_v[i], q, ap);
        double tmp1 = measurement(p, q * (Eigen::Matrix3d::Identity() + skewSymmetric(eps_v[i])), ap) -
                      measurement(p, q * (Eigen::Matrix3d::Identity() - skewSymmetric(eps_v[i])), ap);
        double tmp2 = measurement(p, (Eigen::Quaterniond(q) * Eigen::Quaterniond(1.0, +eps_v[i].x() / 2, +eps_v[i].y() / 2, +eps_v[i].z() / 2)).normalized().toRotationMatrix(), ap) -
                      measurement(p, (Eigen::Quaterniond(q) * Eigen::Quaterniond(1.0, -eps_v[i].x() / 2, -eps_v[i].y() / 2, -eps_v[i].z() / 2)).normalized().toRotationMatrix(), ap);
        printf("tmp1: %f, tmp2: %f\n", tmp1 / (2 * EPS), tmp2 / (2 * EPS));
        ROS_ASSERT(std::fabs(tmp1 - tmp2) / (2 * EPS) < EPS);
        J(0, 6 + i) = tmp1;
        J(0, 9 + i) = measurement(p, q, ap + eps_v[i]) - measurement(p, q, ap - eps_v[i]);
    }
    return J / (2 * EPS);
}

