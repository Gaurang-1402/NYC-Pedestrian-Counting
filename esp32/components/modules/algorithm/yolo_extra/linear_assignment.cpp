#include "linear_assignment.hpp"

Assignment min_cost_matching(
    std::function<Eigen::MatrixXd(const std::vector<Track> &, const std::vector<Detection> &, const IndexList &, const IndexList &)> distance_metric,
    double max_distance,
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const IndexList &track_indices = IndexList(),
    const IndexList &detection_indices = IndexList())
{
      if (track_indices.empty())
            track_indices.resize(tracks.size());
      if (detection_indices.empty())
            detection_indices.resize(detections.size());

      if (detection_indices.empty() || track_indices.empty())
            return Assignment();

      Eigen::MatrixXd cost_matrix = distance_metric(tracks, detections, track_indices, detection_indices);
      cost_matrix = cost_matrix.array().min(max_distance).matrix();

      Eigen::SparseMatrix<double> assignment_matrix = linear_sum_assignment(cost_matrix);

      Assignment matches;
      IndexList unmatched_tracks, unmatched_detections;

      for (int col = 0; col < detection_indices.size(); ++col)
      {
            if (std::find(assignment_matrix.innerIndexPtr(), assignment_matrix.innerIndexPtr() + assignment_matrix.nonZeros(), col) == assignment_matrix.innerIndexPtr() + assignment_matrix.nonZeros())
                  unmatched_detections.push_back(detection_indices[col]);
      }

      for (int row = 0; row < track_indices.size(); ++row)
      {
            if (std::find(assignment_matrix.outerIndexPtr(), assignment_matrix.outerIndexPtr() + assignment_matrix.outerSize(), row) == assignment_matrix.outerIndexPtr() + assignment_matrix.outerSize())
                  unmatched_tracks.push_back(track_indices[row]);
      }

      for (int row = 0; row < assignment_matrix.outerSize(); ++row)
      {
            for (Eigen::SparseMatrix<double>::InnerIterator it(assignment_matrix, row); it; ++it)
            {
                  int track_idx = track_indices[it.row()];
                  int detection_idx = detection_indices[it.col()];
                  if (cost_matrix(it.row(), it.col()) > max_distance)
                  {
                        unmatched_tracks.push_back(track_idx);
                        unmatched_detections.push_back(detection_idx);
                  }
                  else
                  {
                        matches.push_back(std::make_pair(track_idx, detection_idx));
                  }
            }
      }

      return matches;
}

Assignment matching_cascade(
    std::function<Eigen::MatrixXd(const std::vector<Track> &, const std::vector<Detection> &, const IndexList &, const IndexList &)> distance_metric,
    double max_distance,
    int cascade_depth,
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const IndexList &track_indices = IndexList(),
    const IndexList &detection_indices = IndexList())
{
      if (track_indices.empty())
      {
            track_indices.resize(tracks.size());
            std::iota(track_indices.begin(), track_indices.end(), 0);
      }
      if (detection_indices.empty())
      {
            detection_indices.resize(detections.size());
            std::iota(detection_indices.begin(), detection_indices.end(), 0);
      }

      IndexList unmatched_detections = detection_indices;
      Assignment matches;

      for (int level = 0; level < cascade_depth; ++level)
      {
            if (unmatched_detections.empty())
                  break;

            IndexList track_indices_l;
            for (int k : track_indices)
            {
                  if (tracks[k].time_since_update == 1 + level)
                        track_indices_l.push_back(k);
            }

            if (track_indices_l.empty())
                  continue;

            Assignment matches_l = min_cost_matching(
                distance_metric, max_distance, tracks, detections, track_indices_l, unmatched_detections);

            for (const auto &match : matches_l)
                  matches.push_back(match);

            for (const auto &match : matches_l)
            {
                  unmatched_detections.erase(std::remove(unmatched_detections.begin(), unmatched_detections.end(), match.second), unmatched_detections.end());
            }
      }

      IndexList unmatched_tracks;
      for (int k : track_indices)
      {
            bool found = false;
            for (const auto &match : matches)
            {
                  if (k == match.first)
                  {
                        found = true;
                        break;
                  }
            }
            if (!found)
                  unmatched_tracks.push_back(k);
      }

      return matches;
}

Eigen::MatrixXd gate_cost_matrix(
    const KalmanFilter &kf,
    const Eigen::MatrixXd &cost_matrix,
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const IndexList &track_indices,
    const IndexList &detection_indices,
    double gated_cost = INFTY_COST,
    bool only_position = false)
{
      int gating_dim = only_position ? 2 : 4;
      double gating_threshold = kf.chi2inv95[gating_dim];
      Eigen::MatrixXd measurements(detection_indices.size(), only_position ? 2 : 4);

      for (int i = 0; i < detection_indices.size(); ++i)
      {
            const Detection &detection = detections[detection_indices[i]];
            measurements.row(i) = detection.to_xyah().transpose();
      }

      Eigen::MatrixXd gated_cost_matrix = cost_matrix;

      for (int row = 0; row < track_indices.size(); ++row)
      {
            const Track &track = tracks[track_indices[row]];
            Eigen::VectorXd gating_distance = kf.gating_distance(track.mean, track.covariance, measurements, only_position);

            for (int col = 0; col < detection_indices.size(); ++col)
            {
                  if (gating_distance(col) > gating_threshold)
                  {
                        gated_cost_matrix(row, col) = gated_cost;
                  }
            }
      }

      return gated_cost_matrix;
}