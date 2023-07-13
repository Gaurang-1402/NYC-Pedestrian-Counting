#ifndef NN_METRIC_HPP
#define NN_METRIC_HPP

#include <cmath>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>

Eigen::MatrixXd pdist(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b);
Eigen::MatrixXd cosine_distance(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, bool data_is_normalized = false);
Eigen::VectorXd nn_euclidean_distance(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y);
Eigen::VectorXd nn_cosine_distance(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y);

class NearestNeighborDistanceMetric
{
public:
    NearestNeighborDistanceMetric(const std::string &metric, double matching_threshold, int budget = -1);

    void partial_fit(const Eigen::MatrixXd &features, const Eigen::VectorXi &targets, const std::vector<int> &active_targets);
    Eigen::MatrixXd distance(const Eigen::MatrixXd &features, const std::vector<int> &targets);

private:
    std::function<Eigen::VectorXd(const Eigen::MatrixXd &, const Eigen::MatrixXd &)> _metric;
    double matching_threshold;
    int budget;
    std::unordered_map<int, std::vector<Eigen::VectorXd>> samples;
};

#endif // NN_METRIC_HPP
