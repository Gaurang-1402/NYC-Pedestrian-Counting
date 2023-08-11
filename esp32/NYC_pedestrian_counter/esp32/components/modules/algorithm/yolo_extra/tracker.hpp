#include "tracker.hpp"
#include "linear_assignment.hpp"
#include "iou_matching.hpp"

Tracker::Tracker(const NearestNeighborDistanceMetric &metric, double max_iou_distance, int max_age, int n_init)
    : metric_(metric),
      max_iou_distance_(max_iou_distance),
      max_age_(max_age),
      n_init_(n_init),
      kf_(KalmanFilter()),
      tracks_(std::vector<Track>()),
      next_id_(1) {}

void Tracker::predict()
{
      for (auto &track : tracks_)
      {
            track.predict(kf_);
      }
}

void Tracker::update(const std::vector<Detection> &detections)
{
      auto [matches, unmatched_tracks, unmatched_detections] = match(detections);

      for (const auto &match : matches)
      {
            int track_idx = match.first;
            int detection_idx = match.second;
            tracks_[track_idx].update(kf_, detections[detection_idx]);
      }

      for (int track_idx : unmatched_tracks)
      {
            tracks_[track_idx].mark_missed();
      }

      for (int detection_idx : unmatched_detections)
      {
            initiate_track(detections[detection_idx]);
      }

      tracks_.erase(
          std::remove_if(
              tracks_.begin(), tracks_.end(),
              [](const Track &track)
              {
                    return track.is_deleted();
              }),
          tracks_.end());

      std::vector<int> active_targets;
      std::vector<cv::Mat> features;
      std::vector<int> targets;

      for (auto &track : tracks_)
      {
            if (!track.is_confirmed())
            {
                  continue;
            }

            features.insert(features.end(), track.features().begin(), track.features().end());
            targets.insert(targets.end(), track.features().size(), track.track_id());
            track.clear_features();
            active_targets.push_back(track.track_id());
      }

      metric_.partial_fit(features, targets, active_targets);
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, std::vector<int>> Tracker::match(
    const std::vector<Detection> &detections)
{

      auto gated_metric = [&](const std::vector<Track> &tracks, const std::vector<Detection> &dets,
                              const std::vector<int> &track_indices, const std::vector<int> &detection_indices)
      {
            std::vector<cv::Mat> features;
            std::vector<int> targets;

            for (int detection_idx : detection_indices)
            {
                  features.push_back(dets[detection_idx].feature());
            }

            for (int track_idx : track_indices)
            {
                  targets.push_back(tracks[track_idx].track_id());
            }

            cv::Mat cost_matrix = metric_.distance(features, targets);
            cost_matrix = gate_cost_matrix(
                kf_, cost_matrix, tracks, dets, track_indices, detection_indices);

            return cost_matrix;
      };

      std::vector<int> confirmed_tracks;
      std::vector<int> unconfirmed_tracks;

      for (int i = 0; i < tracks_.size(); i++)
      {
            if (tracks_[i].is_confirmed())
            {
                  confirmed_tracks.push_back(i);
            }
            else
            {
                  unconfirmed_tracks.push_back(i);
            }
      }

      auto [matches_a, unmatched_tracks_a, unmatched_detections] = matching_cascade(
          gated_metric, metric_.matching_threshold(), max_age_, tracks_, detections, confirmed_tracks);

      std::vector<int> iou_track_candidates = unconfirmed_tracks;
      for (int k : unmatched_tracks_a)
      {
            if (tracks_[k].time_since_update() == 1)
            {
                  iou_track_candidates.push_back(k);
            }
      }

      unmatched_tracks_a.erase(
          std::remove_if(
              unmatched_tracks_a.begin(), unmatched_tracks_a.end(),
              [&](int k)
              {
                    return tracks_[k].time_since_update() == 1;
              }),
          unmatched_tracks_a.end());

      auto [matches_b, unmatched_tracks_b, unmatched_detections_b] = min_cost_matching(
          iou_matching::iou_cost, max_iou_distance_, tracks_, detections, iou_track_candidates, unmatched_detections);

      std::vector<std::pair<int, int>> matches;
      for (const auto &match : matches_a)
      {
            matches.push_back(match);
      }
      for (const auto &match : matches_b)
      {
            matches.push_back(match);
      }

      std::vector<int> unmatched_tracks;
      for (int k : unmatched_tracks_a)
      {
            unmatched_tracks.push_back(k);
      }
      for (int k : unmatched_tracks_b)
      {
            unmatched_tracks.push_back(k);
      }

      return std::make_tuple(matches, unmatched_tracks, unmatched_detections);
}

void Tracker::initiate_track(const Detection &detection)
{
      cv::Mat mean, covariance;
      kf_.initiate(detection.to_xyah(), mean, covariance);
      std::string class_name = detection.get_class();
      tracks_.push_back(
          Track(mean, covariance, next_id_, n_init_, max_age_, detection.feature(), class_name));
      next_id_++;
}
