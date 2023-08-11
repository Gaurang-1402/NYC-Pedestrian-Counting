#include "kalman_filter.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

/*
Table for the 0.95 quantile of the chi-square distribution with N degrees of
freedom (contains values for N=1, ..., 9). Taken from MATLAB/Octave's chi2inv
function and used as Mahalanobis gating threshold.
*/
static std::map<int, double> chi2inv95 = {
    {1, 3.8415},
    {2, 5.9915},
    {3, 7.8147},
    {4, 9.4877},
    {5, 11.070},
    {6, 12.592},
    {7, 14.067},
    {8, 15.507},
    {9, 16.919}};

KalmanFilter::KalmanFilter()
{
      int ndim = 4;
      double dt = 1.0;

      // Create Kalman filter model matrices.
      motion_mat = Eigen::MatrixXd::Identity(2 * ndim, 2 * ndim);
      for (int i = 0; i < ndim; ++i)
      {
            motion_mat(i, ndim + i) = dt;
      }
      update_mat = Eigen::MatrixXd::Identity(ndim, 2 * ndim);

      // Motion and observation uncertainty are chosen relative to the current
      // state estimate. These weights control the amount of uncertainty in
      // the model. This is a bit hacky.
      std_weight_position = 1.0 / 20;
      std_weight_velocity = 1.0 / 160;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KalmanFilter::initiate(const Eigen::VectorXd &measurement)
{
      Eigen::VectorXd mean_pos = measurement;
      Eigen::VectorXd mean_vel = Eigen::VectorXd::Zero(mean_pos.size());
      Eigen::VectorXd mean(mean_pos.size() + mean_vel.size());
      mean << mean_pos, mean_vel;

      std::vector<double> std{
          2 * std_weight_position * measurement(3),
          2 * std_weight_position * measurement(3),
          1e-2,
          2 * std_weight_position * measurement(3),
          10 * std_weight_velocity * measurement(3),
          10 * std_weight_velocity * measurement(3),
          1e-5,
          10 * std_weight_velocity * measurement(3)};

      Eigen::MatrixXd centered = measurement.rowwise() - measurement.colwise().mean();
      Eigen::MatrixXd covariance = (centered.adjoint() * centered) / double(measurement.rows() - 1);
      Eigen::MatrixXd covariance = std::pow(std, 2).asDiagonal();

      return std::make_pair(mean, covariance);
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KalmanFilter::predict(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
{
      Eigen::VectorXd predicted_mean = motion_mat * mean;
      Eigen::MatrixXd predicted_covariance = motion_mat * covariance * motion_mat.transpose();

      return std::make_pair(predicted_mean, predicted_covariance);
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KalmanFilter::project(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
{
      std::vector<double> std{
          std_weight_position * mean(3),
          std_weight_position * mean(3),
          1e-1,
          std_weight_position * mean(3)};
      Eigen::MatrixXd innovation_cov = std::pow(std, 2).asDiagonal();

      Eigen::VectorXd projected_mean = update_mat * mean;
      Eigen::MatrixXd projected_cov = update_mat * covariance * update_mat.transpose();

      return std::make_pair(projected_mean, projected_cov + innovation_cov);
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KalmanFilter::update(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance, const Eigen::VectorXd &measurement)
{

      std::pair<Eigen::VectorXd, Eigen::MatrixXd> projected = project(mean, covariance);
      Eigen::VectorXd projected_mean = projected.first;
      Eigen::MatrixXd projected_cov = projected.second;

      Eigen::LLT<Eigen::MatrixXd> chol_factor(projected_cov);
      Eigen::MatrixXd lower = chol_factor.matrixL();
      Eigen::VectorXd kalman_gain = (chol_factor.solve(covariance * update_mat.transpose().transpose())).transpose();
      Eigen::VectorXd innovation = measurement - projected_mean;

      Eigen::VectorXd new_mean = mean + kalman_gain * innovation;
      Eigen::MatrixXd new_covariance = covariance - kalman_gain * projected_cov * kalman_gain.transpose();

      return std::make_pair(new_mean, new_covariance);
}

Eigen::VectorXd KalmanFilter::gating_distance(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance, const Eigen::MatrixXd &measurements, bool only_position)
{
      std::pair<Eigen::VectorXd, Eigen::MatrixXd> projected = project(mean, covariance);
      Eigen::VectorXd projected_mean = projected.first;
      Eigen::MatrixXd projected_cov = projected.second;

      if (only_position)
      {
            projected_mean = projected_mean.head(2);
            projected_cov = projected_cov.topLeftCorner(2, 2);
            measurements = measurements.leftCols(2);
      }

      Eigen::LLT<Eigen::MatrixXd> chol_factor(projected_cov);
      Eigen::MatrixXd cholesky_factor = chol_factor.matrixL();
      Eigen::MatrixXd d = measurements - projected_mean.transpose();
      Eigen::MatrixXd z = cholesky_factor.triangularView<Eigen::Lower>().solve(d.transpose()).transpose();
      Eigen::VectorXd squared_maha = z.rowwise().squaredNorm();

      return squared_maha;
}
