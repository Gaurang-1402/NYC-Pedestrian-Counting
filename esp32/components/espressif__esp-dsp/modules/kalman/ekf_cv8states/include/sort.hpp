#ifndef SORT_HPP
#define SORT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>

#include "esp_timer.h"

// Assume tracking_utils.h and kalman_box_tracker.h are defined with necessary functions

#include "ekf_helper.hpp"

// Implement the C++ version of the Sort class
class Sort
{
public:
      Sort(int max_age = 2, int min_hits = 3) : max_age(max_age), min_hits(min_hits), frame_count(0) {}

      std::vector<std::vector<double>> update(const std::vector<std::vector<double>> &dets, float s_time)
      {
            frame_count++;
            std::vector<std::vector<double>> trks;
            // std::vector<std::vector<double>> trks(trackers.size(), std::vector<double>(5, 0));
            std::vector<int> to_del;
            std::vector<std::vector<double>> ret;

            for (size_t t = 0; t < trackers.size(); ++t)
            {
                  float dt = (esp_timer_get_time() - s_time) / 10000000.0f;
                  std::vector<double> pos = trackers[t].predict(dt);
                  trks.push_back({pos[0], pos[1], pos[2], pos[3], 0});

                  if (std::isnan(pos[0]) || std::isnan(pos[1]) || std::isnan(pos[2]) || std::isnan(pos[3]))
                  {
                        to_del.push_back(t);
                  }
            }

            std::vector<std::vector<double>> temp_trks;
            temp_trks.reserve(trks.size() - to_del.size());
            for (size_t i = 0; i < trks.size(); ++i)
            {
                  if (std::find(to_del.begin(), to_del.end(), static_cast<int>(i)) == to_del.end())
                  {
                        temp_trks.emplace_back(std::move(trks[i]));
                  }
            }

            trks = std::move(temp_trks);

            trackers.erase(
                std::remove_if(trackers.begin(), trackers.end(), [&](const KalmanBoxTracker &tracker)
                               {
        size_t index = &tracker - &trackers[0];
        return std::find(to_del.begin(), to_del.end(), static_cast<int>(index)) != to_del.end(); }),
                trackers.end());

            auto [matched, unmatched] =
                associate_detections_to_trackers(dets, trks);

            auto &[unmatched_dets, unmatched_trks] = unmatched;

            for (size_t t = 0; t < trackers.size(); ++t)
            {
                  if (std::find(unmatched_trks.begin(), unmatched_trks.end(), static_cast<int>(t)) == unmatched_trks.end())
                  {
                        auto x = [t](const std::vector<int> &pair)
                        { return pair[1] == static_cast<int>(t); };

                        int d = matched[static_cast<size_t>(std::find_if(matched.begin(), matched.end(), x) - matched.begin())][0];

                        if (!dets[d].empty())
                        {
                              float dt = (esp_timer_get_time() - s_time) / 10000000.0f;
                              trackers[t].update(dets[d], dt);
                        }
                  }
            }

            for (auto i : unmatched_dets)
            {
                  KalmanBoxTracker tracker(dets[i]);
                  trackers.emplace_back(tracker);
            }

            const size_t tracker_size = trackers.size();

            std::vector<KalmanBoxTracker> temp_trackers2;

            for (size_t i = 0; i < tracker_size; ++i)
            {
                  std::vector<double> d = trackers[i].get_state();
                  if ((trackers[i].time_since_update < 5) && (trackers[i].hit_streak >= min_hits - 1 || frame_count <= min_hits))
                  {
                        ret.emplace_back(std::vector<double>(d.begin(), d.end()));
                        ret.back().push_back(trackers[i].id + 1);
                  }
                  if (trackers[i].time_since_update <= max_age)
                  {
                        temp_trackers2.emplace_back(trackers[i]);
                  }
            }
            trackers = std::move(temp_trackers2);

            return ret;
      }

      std::vector<KalmanBoxTracker>
          trackers;

private:
      int max_age;
      int min_hits;
      int frame_count;
};

#endif // SORT_HPP