#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <eigen-3.4.0/Eigen/Dense>
class KalmanFilter
{
public:
      KalmanFilter();

      std::pair<Eigen::VectorXd, Eigen::MatrixXd> initiate(const Eigen::VectorXd &measurement);

      std::pair<Eigen::VectorXd, Eigen::MatrixXd> predict(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance);

      std::pair<Eigen::VectorXd, Eigen::MatrixXd> update(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance, const Eigen::VectorXd &measurement);

      Eigen::VectorXd gating_distance(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance, const Eigen::MatrixXd &measurements, bool only_position = false);

private:
      Eigen::MatrixXd motion_mat;
      Eigen::MatrixXd update_mat;
      double std_weight_position;
      double std_weight_velocity;
};

#endif // KALMAN_FILTER_HPP
