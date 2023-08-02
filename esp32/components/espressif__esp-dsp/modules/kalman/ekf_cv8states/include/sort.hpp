#ifndef SORT_HPP
#define SORT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

// Assume tracking_utils.h and kalman_box_tracker.h are defined with necessary functions

#include "ekf_helper.hpp"

// Implement the C++ version of the Sort class
class Sort
{
public:
      Sort(int max_age = 1, int min_hits = 3) : max_age(max_age), min_hits(min_hits), frame_count(0) {}

      std::vector<std::vector<double>> update(const std::vector<std::vector<double>> &dets)
      {
            frame_count++;
            std::vector<std::vector<double>> trks(trackers.size(), std::vector<double>(5, 0));
            std::vector<int> to_del;
            std::vector<std::vector<double>> ret;

            for (size_t t = 0; t < trackers.size(); ++t)
            {
                  std::vector<double> pos = trackers[t].predict();
                  trks[t][0] = pos[0];
                  trks[t][1] = pos[1];
                  trks[t][2] = pos[2];
                  trks[t][3] = pos[3];
                  trks[t][4] = 0;
                  if (std::any_of(pos.begin(), pos.end(), [](double val)
                                  { return std::isnan(val); }))
                  {
                        to_del.push_back(t);
                  }
            }

            trks.erase(std::remove_if(trks.begin(), trks.end(), [](const std::vector<double> &trk)
                                      { return std::all_of(trk.begin(), trk.end(), [](double val)
                                                           { return std::isnan(val); }); }),
                       trks.end());

            for (auto it = to_del.rbegin(); it != to_del.rend(); ++it)
            {
                  trackers.erase(trackers.begin() + *it);
            }

            // Call the function for association between detections and trackers (similar to Python version)

            // Return <Matches, <unmatched_detections, unmatched_trackers>>

            std::pair<std::vector<std::vector<int>>, std::pair<std::vector<int>, std::vector<int>>> matched_unmatched =
                associate_detections_to_trackers(dets, trks);
            std::vector<std::vector<int>> matched = matched_unmatched.first;
            std::vector<int> unmatched_dets = matched_unmatched.second.first, unmatched_trks = matched_unmatched.second.second;

            for (size_t t = 0; t < trackers.size(); ++t)
            {
                  if (std::find(unmatched_trks.begin(), unmatched_trks.end(), static_cast<int>(t)) == unmatched_trks.end())
                  {
                        auto x = [t](const std::vector<int> &pair)
                        { return pair[1] == static_cast<int>(t); };

                        int d = matched[static_cast<size_t>(std::find_if(matched.begin(), matched.end(), x) - matched.begin())][0];

                        if (!dets[d].empty())
                        {
                              trackers[t].update(dets[d]);
                        }
                  }
            }

            for (auto i : unmatched_dets)
            {
                  trackers.emplace_back(dets[i]);
            }

            size_t i = trackers.size();
            for (auto it = trackers.rbegin(); it != trackers.rend(); ++it)
            {
                  std::vector<double> d = it->get_state();
                  if ((it->time_since_update < 1) && (it->hit_streak >= min_hits || frame_count <= min_hits))
                  {
                        ret.emplace_back(std::vector<double>(d.begin(), d.end()));
                        ret.back().push_back(it->id + 1);
                  }
                  --i;
                  if (it->time_since_update > max_age)
                  {
                        trackers.erase(trackers.begin() + i);
                  }
            }

            return ret;
      }

private:
      int max_age;
      int min_hits;
      int frame_count;
      std::vector<KalmanBoxTracker> trackers;
};

#endif // SORT_HPP