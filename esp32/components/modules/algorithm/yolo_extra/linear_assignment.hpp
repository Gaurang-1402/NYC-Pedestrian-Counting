#ifndef LINEAR_ASSIGNMENT_HPP
#define LINEAR_ASSIGNMENT_HPP

#include <vector>
#include <functional>
#include <algorithm>
#include <eigen-3.4.0/Eigen/Dense>#include <Eigen/Sparse>
#include "detection.hpp"

// struct Detection
// {
//     Eigen::VectorXd to_xyah() const;
// };

class KalmanFilter
{
public:
    double chi2inv95[5];
    Eigen::VectorXd gating_distance(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance,
                                    const Eigen::MatrixXd &measurements, bool only_position) const;
};

typedef std::vector<std::pair<int, int>> Assignment;
typedef std::vector<int> IndexList;

Assignment min_cost_matching(
    std::function<Eigen::MatrixXd(const std::vector<Track> &, const std::vector<Detection> &, const IndexList &, const IndexList &)> distance_metric,
    double max_distance,
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const IndexList &track_indices = IndexList(),
    const IndexList &detection_indices = IndexList());

Assignment matching_cascade(
    std::function<Eigen::MatrixXd(const std::vector<Track> &, const std::vector<Detection> &, const IndexList &, const IndexList &)> distance_metric,
    double max_distance,
    int cascade_depth,
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const IndexList &track_indices = IndexList(),
    const IndexList &detection_indices = IndexList());

Eigen::MatrixXd gate_cost_matrix(
    const KalmanFilter &kf,
    const Eigen::MatrixXd &cost_matrix,
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const IndexList &track_indices,
    const IndexList &detection_indices,
    double gated_cost = 1e+5,
    bool only_position = false);

#endif // LINEAR_ASSIGNMENT_HPP
