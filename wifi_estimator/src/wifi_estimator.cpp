#include "wifi_estimator.h"

template <typename Derived>
Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const MatrixBase<Derived> &q)
{
    Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << 0, -q(2), q(1),
        q(2), 0, -q(0),
        -q(1), q(0), 0;
    return ans;
}


WiFiEstimator::WiFiEstimator():
    frame_count(0), current_time(-1),
    A(NUMBER_OF_STATE, NUMBER_OF_STATE), b(NUMBER_OF_STATE)
{
    odometry[0](0) = 5;
    odometry[0](1) = 5;
    //odometry[0](0) = 1;
    //odometry[0](1) = 1;
    odometry[0](2) = 1.0;
    //odometry[0](3) = 0;
    //odometry[0](4) = 0;
    //odometry[0](5) = 0;
    //odometry[0](6) = 0.6;
    //odometry[0](7) = -1.05;
    //odometry[0](8) = -9.65;
    //odometry[0].tail<3>() *= 1.01;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        IMU_linear[i].setZero();
        IMU_angular[i].setIdentity();
        IMU_cov[i].setZero();
        IMU_cov_nl[i].setZero();
        wifi_measurement[i].resize(NUMBER_OF_AP);
    }
    Rs[0].setIdentity();
    A.setZero();
    b.setZero();
    //A.topLeftCorner<3, 3>() = Matrix3d::Identity() * 10000;
    A.topLeftCorner<3, 3>() = Matrix3d::Identity() * 1;
    //b.head<3>() = odometry[0].head<3>() * 10000;
    b.head<3>() = odometry[0].head<3>() * 1;
}

void WiFiEstimator::processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;
    if (frame_count != 0)
    {
        Quaterniond q(IMU_angular[frame_count]);
        Quaterniond dq(1,
                       angular_velocity(0) * dt / 2,
                       angular_velocity(1) * dt / 2,
                       angular_velocity(2) * dt / 2);
        dq.w() = 1 - dq.vec().transpose() * dq.vec();
        IMU_angular[frame_count] = (q * dq).normalized();

        Rs[frame_count] = Rs[frame_count - 1] * IMU_angular[frame_count];
        //This is the integration for position, velocity and time for each frame
        IMU_linear[frame_count].segment<3>(0) += IMU_linear[frame_count].segment<3>(3) * dt + IMU_angular[frame_count] * linear_acceleration * dt * dt / 2;
        IMU_linear[frame_count].segment<3>(3) += IMU_angular[frame_count] * linear_acceleration * dt;
        IMU_linear[frame_count](6) += dt;

        {
            Matrix<double, 6, 6> F = Matrix<double, 6, 6>::Identity();
            F.block<3, 3>(0, 3) = dt * Matrix3d::Identity();

            Matrix<double, 6, 3> G = Matrix<double, 6, 3>::Zero();
            G.block<3, 3>(0, 0) = 0.5 * dt * dt * Matrix3d::Identity();
            G.block<3, 3>(3, 0) = dt * Matrix3d::Identity();

            IMU_cov[frame_count] = F * IMU_cov[frame_count] * F.transpose() + G * acc_cov * G.transpose();
        }
    }
}

SolutionContainer WiFiEstimator::processWiFi(const vector<pair<int, Vector3d>> &wifi)
{
    ROS_INFO("Adding AP measurements %lu", wifi.size());
    ROS_INFO("Solving %d", frame_count);
    ROS_INFO_STREAM("IMU_linear[" << frame_count << "]: " << IMU_linear[frame_count]);
    //ROS_ERROR("imu  degree: %f", 180.0 / M_PI * std::atan2(-Rs[frame_count].col(0).y(), -Rs[frame_count].col(0).x()));
    //ROS_ERROR("wifi degree: %f", 180.0 / M_PI * std::atan2(wifi[0].second.y(), wifi[0].second.x()));

    for (int i = 0; i < int(wifi.size()); i++)
    {
		//This is for data generator and real experiments
        wifi_measurement[frame_count][wifi[i].first] = wifi[i].second;
		//This is for imu reading simulation.
        //wifi_measurement[frame_count][wifi[i].first] = -Rs[frame_count].col(0);
        //wifi_measurement[frame_count][wifi[i].first].z() = 0;
    }

    SolutionContainer solution = solveOdometry();
    marginalize();
    return solution;
}

SolutionContainer WiFiEstimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return SolutionContainer();
    else
        solveOdometryLinear();

    for (int i = 0; i <= frame_count; i++)
        odometry[i] = x.segment<9>(i * 9);

    SolutionContainer solution_container;
    solution_container.p = odometry[frame_count].segment<3>(0);
    solution_container.v = odometry[frame_count].segment<3>(3);
    solution_container.g = odometry[frame_count].segment<3>(6);
    solution_container.q = Rs[frame_count];
    for (int i = 0; i < NUMBER_OF_AP; i++)
        solution_container.p_ap[i] = x.tail<3>();
    return solution_container;
}

void WiFiEstimator::solveOdometryLinear()
{
    for (int i = 0; i < frame_count; i++)
    {
        MatrixXd tmp_A(9, 18);
        tmp_A.setZero();
        VectorXd tmp_b(9);
        tmp_b.setZero();

        double dt = IMU_linear[i + 1](6);
        ROS_DEBUG("%d dt: %lf", i, dt);
        tmp_A.block<3, 3>(0, 0 * 9)           = -Rs[i].inverse();
        tmp_A.block<3, 3>(0, 0 * 9 + 3)       = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 0 * 9 + 6)       = dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, (0 + 1) * 9)     = Rs[i].inverse();
        tmp_b.segment<3>(0)                   = IMU_linear[i + 1].segment<3>(0);

        tmp_A.block<3, 3>(3, (0 + 1) * 9 + 3) = Rs[i].inverse() * Rs[i + 1];
        tmp_A.block<3, 3>(3, 0 * 9 + 3)       = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 0 * 9 + 6)       = dt * Matrix3d::Identity();
        tmp_b.segment<3>(3)                   = IMU_linear[i + 1].segment<3>(3);

        tmp_A.block<3, 3>(6, (0 + 1) * 9 + 6) = Rs[i].inverse() * Rs[i + 1];
        tmp_A.block<3, 3>(6, 0 * 9 + 6)       = -Matrix3d::Identity();
        tmp_b.segment<3>(6)                   = Vector3d::Zero();

        Matrix<double, 9, 9> cov = Matrix<double, 9, 9>::Zero();
        cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        cov.block<3, 3>(6, 6) = gra_cov;

        MatrixXd cov_inv = cov.inverse();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
        A.block<18, 18>(i * 9, i * 9) += r_A;
        b.segment<18>(i * 9)          += r_b;
    }

    for (int i = 0; i <= frame_count; i++)
        for (int j = 0; j < NUMBER_OF_AP; j++)
        {
            ROS_DEBUG_STREAM("sar: " << wifi_measurement[i][j].transpose());

            MatrixXd tmp_A(3, 6);
            tmp_A.setZero();
			double weight = 1;
            Matrix3d reduce = skewSymmetric(wifi_measurement[i][j]);
            //tmp_A.block<3, 3>(0, 0) = 10000 * reduce * -Matrix3d::Identity();
            tmp_A.block<3, 3>(0, 0) = weight * reduce * -Matrix3d::Identity();
            //tmp_A.block<3, 3>(0, 3) = 10000 * reduce * Matrix3d::Identity();
            tmp_A.block<3, 3>(0, 3) = weight * reduce * Matrix3d::Identity();
            MatrixXd r_A = tmp_A.transpose() * tmp_A;
            int ii = i * 9, jj = (frame_count + 1) * 9 + j * 3;
            A.block<3, 3>(ii, ii) += r_A.block<3, 3>(0, 0);
            A.block<3, 3>(ii, jj) += r_A.block<3, 3>(0, 3);
            A.block<3, 3>(jj, ii) += r_A.block<3, 3>(3, 0);
            A.block<3, 3>(jj, jj) += r_A.block<3, 3>(3, 3);
        }
    x = A.llt().solve(b);
}

void WiFiEstimator::marginalize()
{
    if (frame_count < WINDOW_SIZE)
    {
        frame_count++;
        return;
    }

    int m = 9, n = NUMBER_OF_STATE - 9;
    MatrixXd Amm_inv = A.block(0, 0, m, m).inverse();
    VectorXd bmm = b.segment(0, m);
    MatrixXd Amr = A.block(0, m, m, n);
    MatrixXd Arm = A.block(m, 0, n, m);
    MatrixXd Arr = A.block(m, m, n, n);
    VectorXd brr = b.segment(m, n);
    MatrixXd Ap = Arr - Arm * Amm_inv * Amr;
    VectorXd bp = brr - Arm * Amm_inv * bmm;

    A.setZero();
    b.setZero();
    A.topLeftCorner(WINDOW_SIZE * 9, WINDOW_SIZE * 9)       = Ap.topLeftCorner(WINDOW_SIZE * 9, WINDOW_SIZE * 9);
    A.topRightCorner(WINDOW_SIZE * 9, NUMBER_OF_AP * 3)     = Ap.topRightCorner(WINDOW_SIZE * 9, NUMBER_OF_AP * 3);
    A.bottomLeftCorner(NUMBER_OF_AP * 3, WINDOW_SIZE * 9)   = Ap.bottomLeftCorner(NUMBER_OF_AP * 3, WINDOW_SIZE * 9);
    A.bottomRightCorner(NUMBER_OF_AP * 3, NUMBER_OF_AP * 3) = Ap.bottomRightCorner(NUMBER_OF_AP * 3, NUMBER_OF_AP * 3);

    b.head(WINDOW_SIZE * 9) = bp.head(WINDOW_SIZE * 9);
    b.tail(NUMBER_OF_AP * 3) = bp.tail(NUMBER_OF_AP * 3);


    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        IMU_linear[i].swap(IMU_linear[i + 1]);
        IMU_angular[i].swap(IMU_angular[i + 1]);
        wifi_measurement[i].swap(wifi_measurement[i + 1]);
        IMU_cov[i].swap(IMU_cov[i + 1]);
        IMU_cov_nl[i].swap(IMU_cov_nl[i + 1]);
        odometry[i].swap(odometry[i + 1]);
        Rs[i].swap(Rs[i + 1]);
    }
    IMU_linear[WINDOW_SIZE].setZero();
    IMU_angular[WINDOW_SIZE].setIdentity();
    IMU_cov[WINDOW_SIZE].setZero();
    IMU_cov_nl[WINDOW_SIZE].setZero();
}



