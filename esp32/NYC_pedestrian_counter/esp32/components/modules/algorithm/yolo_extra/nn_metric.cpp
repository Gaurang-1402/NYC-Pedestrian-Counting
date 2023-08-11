#include "nn_metric.hpp"

Eigen::MatrixXd pdist(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b)
{
      Eigen::MatrixXd a2 = a.array().square().rowwise().sum();
      Eigen::MatrixXd b2 = b.array().square().rowwise().sum();
      Eigen::MatrixXd r2 = -2.0 * a * b.transpose() + a2.replicate(1, b.rows()) + b2.transpose().replicate(a.rows(), 1);
      r2 = r2.cwiseMax(0.0);
      return r2;
}

Eigen::MatrixXd cosine_distance(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, bool data_is_normalized)
{
      Eigen::MatrixXd a_normalized = data_is_normalized ? a : a.rowwise().normalized();
      Eigen::MatrixXd b_normalized = data_is_normalized ? b : b.rowwise().normalized();
      return 1.0 - a_normalized * b_normalized.transpose();
}

Eigen::VectorXd nn_euclidean_distance(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y)
{
      Eigen::MatrixXd distances = pdist(x, y);
      return distances.cwiseMin(0.0).colwise().minCoeff();
}

Eigen::VectorXd nn_cosine_distance(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y)
{
      Eigen::MatrixXd distances = cosine_distance(x, y);
      return distances.colwise().minCoeff();
}

NearestNeighborDistanceMetric::NearestNeighborDistanceMetric(const std::string &metric, double matching_threshold, int budget)
    : matching_threshold(matching_threshold), budget(budget)
{
      if (metric == "euclidean")
      {
            _metric = nn_euclidean_distance;
      }
      else if (metric == "cosine")
      {
            _metric = nn_cosine_distance;
      }
      else
      {
            throw std::invalid_argument("Invalid metric; must be either 'euclidean' or 'cosine'");
      }
}

void NearestNeighborDistanceMetric::partial_fit(const Eigen::MatrixXd &features, const Eigen::VectorXi &targets, const std::vector<int> &active_targets)
{
      for (int i = 0; i < features.rows(); ++i)
      {
            int target = targets(i);
            samples[target].push_back(features.row(i));
            if (budget >= 0 && samples[target].size() > budget)
            {
                  samples[target].erase(samples[target].begin());
            }
      }
      for (auto it = samples.begin(); it != samples.end();)
      {
            if (std::find(active_targets.begin(), active_targets.end(), it->first) == active_targets.end())
            {
                  it = samples.erase(it);
            }
            else
            {
                  ++it;
            }
      }
}

Eigen::MatrixXd NearestNeighborDistanceMetric::distance(const Eigen::MatrixXd &features, const std::vector<int> &targets)
{
      Eigen::MatrixXd cost_matrix(targets.size(), features.rows());
      for (int i = 0; i < targets.size(); ++i)
      {
            int target = targets[i];
            cost_matrix.row(i) = _metric(samples[target], features);
      }
      return cost_matrix;
}
