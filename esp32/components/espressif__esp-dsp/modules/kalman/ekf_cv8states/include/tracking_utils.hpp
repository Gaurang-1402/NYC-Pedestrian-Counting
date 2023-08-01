#ifndef TRACKING_UTILS_HPP
#define TRACKING_UTILS_HPP

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include "hungarian.hpp"

using namespace std;

double iou(vector<double> bb_test, vector<double> bb_gt)
{
      double xx1 = max(bb_test[0], bb_gt[0]);
      double yy1 = max(bb_test[1], bb_gt[1]);
      double xx2 = min(bb_test[2], bb_gt[2]);
      double yy2 = min(bb_test[3], bb_gt[3]);
      double w = max(0.0, xx2 - xx1);
      double h = max(0.0, yy2 - yy1);
      double wh = w * h;
      double o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1]) + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh);
      return o;
}

vector<double> convert_bbox_to_z(vector<double> bbox)
{
      double w = bbox[2] - bbox[0];
      double h = bbox[3] - bbox[1];
      double x = bbox[0] + w / 2.0;
      double y = bbox[1] + h / 2.0;
      double s = w * h;
      double r = w / h;
      return vector<double>{x, y, s, r};
}

vector<double> convert_x_to_bbox(vector<double> x, double score = numeric_limits<double>::quiet_NaN())
{
      double w = sqrt(x[2] * x[3]);
      double h = x[2] / w;
      if (isnan(score))
      {
            return vector<double>{x[0] - w / 2.0, x[1] - h / 2.0, x[0] + w / 2.0, x[1] + h / 2.0};
      }
      else
      {
            return vector<double>{x[0] - w / 2.0, x[1] - h / 2.0, x[0] + w / 2.0, x[1] + h / 2.0, score};
      }
}

// A simple implementation of the Hungarian algorithm is needed here.
// Let's assume it's implemented in a function called HungarianAlgorithm.

pair<vector<vector<int>>, pair<vector<int>, vector<int>>>
associate_detections_to_trackers(vector<vector<double>> detections, vector<vector<double>> trackers, double iou_threshold = 0.3)
{
      if (trackers.empty())
      {
            return {vector<vector<int>>{}, {vector<int>(detections.size()), {}}};
      }

      vector<vector<double>> iou_matrix(detections.size(), vector<double>(trackers.size()));

      for (int d = 0; d < detections.size(); d++)
      {
            for (int t = 0; t < trackers.size(); t++)
            {
                  iou_matrix[d][t] = iou(detections[d], trackers[t]);
            }
      }

      LinearSumAssignment solver;

      std::pair<std::vector<int>, std::vector<int>> matched_indices = solver.solve(iou_matrix);

      vector<int> unmatched_detections;
      for (int d = 0; d < detections.size(); d++)
      {
            if (find(matched_indices.first.begin(), matched_indices.first.end(), d) == matched_indices.first.end())
            {
                  unmatched_detections.push_back(d);
            }
      }

      vector<int> unmatched_trackers;
      for (int t = 0; t < trackers.size(); t++)
      {
            if (find(matched_indices.second.begin(), matched_indices.second.end(), t) == matched_indices.second.end())
            {
                  unmatched_trackers.push_back(t);
            }
      }

      vector<vector<int>> matches;
      for (int m = 0; m < matched_indices.first.size(); m++)
      {
            if (iou_matrix[matched_indices.first[m]][matched_indices.second[m]] < iou_threshold)
            {
                  unmatched_detections.push_back(matched_indices.first[m]);
                  unmatched_trackers.push_back(matched_indices.second[m]);
            }
            else
            {
                  matches.push_back(vector<int>{matched_indices.first[m], matched_indices.second[m]});
            }
      }

      return {matches, {unmatched_detections, unmatched_trackers}};
}

#endif // TRACKING_UTILS_HPP