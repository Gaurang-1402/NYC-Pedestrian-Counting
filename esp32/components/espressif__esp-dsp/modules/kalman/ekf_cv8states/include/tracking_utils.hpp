#ifndef TRACKING_UTILS_HPP
#define TRACKING_UTILS_HPP

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include "hungarian.hpp"

using namespace std;

double iou(const vector<double> &bb_test, const vector<double> &bb_gt)
{

      // TL_x, TL_y, w, h
      double bb_test_width = bb_test[2];
      double bb_test_height = bb_test[3];
      double bb_gt_width = bb_gt[2];
      double bb_gt_height = bb_gt[3];

      double xx1 = max(bb_test[0], bb_gt[0]);
      double yy1 = max(bb_test[1], bb_gt[1]);
      double xx2 = min(bb_test[2] + bb_test[0], bb_gt[2] + bb_gt[0]);
      double yy2 = min(bb_test[3] + bb_test[1], bb_gt[3] + bb_gt[1]);

      double w = max(0.0, xx2 - xx1 + 1);
      double h = max(0.0, yy2 - yy1 + 1);
      double interArea = w * h;

      double bb_test_area = (bb_test_width + 1) * (bb_test_height + 1);
      double bb_gt_area = (bb_gt_width + 1) * (bb_gt_height + 1);

      double unionArea = (abs(bb_test_area + bb_gt_area - interArea) + numeric_limits<double>::epsilon());

      double iou = (unionArea > 0) ? interArea / unionArea : 0;

      printf("iou: %f\n", iou);

      return iou;
}

vector<double> convert_bbox_to_z(const vector<double> &bbox)
{
      double w = bbox[2] - bbox[0];
      double h = bbox[3] - bbox[1];
      double x = bbox[0] + w / 2.0;
      double y = bbox[1] + h / 2.0;
      double s = w * h;
      double r = w / h;
      return {x, y, s, r}; // Use emplace_back directly to construct the vector
}

vector<double> convert_x_to_bbox(const vector<double> &x, double score = numeric_limits<double>::quiet_NaN())
{
      double wh = sqrt(x[2] * x[3]);
      double half_w = wh / 2.0;
      double half_h = x[2] / (2.0 * half_w);

      vector<double> bbox;
      bbox.reserve(4); // Reserve memory for the result vector (4 elements)

      bbox.push_back(x[0] - half_w);
      bbox.push_back(x[1] - half_h);
      bbox.push_back(x[0] + half_w);
      bbox.push_back(x[1] + half_h);

      if (!isnan(score))
      {
            bbox.push_back(score);
      }

      return bbox;
}

pair<vector<vector<int>>, pair<vector<int>, vector<int>>>
associate_detections_to_trackers(const vector<vector<double>> &detections, const vector<vector<double>> &trackers, double iou_threshold = 0.3)
{
      if (trackers.empty())
      {
            return {vector<vector<int>>{}, {vector<int>(detections.size()), {}}};
      }

      const size_t num_detections = detections.size();
      const size_t num_trackers = trackers.size();
      vector<vector<double>> iou_matrix(num_detections, vector<double>(num_trackers));

      for (size_t d = 0; d < num_detections; ++d)
      {
            for (size_t t = 0; t < num_trackers; ++t)
            {
                  iou_matrix[d][t] = iou(detections[d], trackers[t]);
            }
      }

      LinearSumAssignment solver;

      std::pair<std::vector<int>, std::vector<int>> matched_indices = solver.solve(iou_matrix);

      // print
      // for (size_t i = 0; i < matched_indices.first.size(); ++i)
      // {
      //       printf("matched_indices: %d, %d\n", matched_indices.first[i], matched_indices.second[i]);
      // }

      std::unordered_set<int> matched_detections(matched_indices.first.begin(), matched_indices.first.end());
      std::unordered_set<int> matched_trackers(matched_indices.second.begin(), matched_indices.second.end());

      const size_t num_matched_detections = matched_indices.first.size();
      const size_t num_matched_trackers = matched_indices.second.size();

      vector<int> unmatched_detections;
      unmatched_detections.reserve(num_detections);
      for (size_t d = 0; d < num_detections; ++d)
      {
            if (matched_detections.find(d) == matched_detections.end())
            {
                  unmatched_detections.push_back(d);
            }
      }

      vector<int> unmatched_trackers;
      unmatched_trackers.reserve(num_trackers);
      for (size_t t = 0; t < num_trackers; t++)
      {
            if (matched_trackers.find(t) == matched_trackers.end())
            {
                  unmatched_trackers.push_back(t);
            }
      }

      vector<vector<int>> matches;
      matches.reserve(num_matched_detections);

      // matched detection
      for (size_t m = 0; m < matched_indices.first.size(); ++m)
      {
            int d = matched_indices.first[m];
            int t = matched_indices.second[m];
            double iou_value = iou_matrix[d][t];

            if (iou_value < iou_threshold)
            {
                  unmatched_detections.push_back(d);
                  unmatched_trackers.push_back(t);
            }
            else
            {
                  matches.push_back(vector<int>{d, t});
            }
      }

      return {matches, {unmatched_detections, unmatched_trackers}};
}

#endif // TRACKING_UTILS_HPP