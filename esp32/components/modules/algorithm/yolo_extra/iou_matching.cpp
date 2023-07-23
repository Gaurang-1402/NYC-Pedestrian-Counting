#include "iou_matching.hpp"
#include <algorithm>
#include <cmath>

double IoU::calculateIntersectionOverUnion(const BoundingBox &bbox, const BoundingBox &candidate)
{
      double bbox_tl_x = bbox.x;
      double bbox_tl_y = bbox.y;
      double bbox_br_x = bbox.x + bbox.width;
      double bbox_br_y = bbox.y + bbox.height;

      double candidate_tl_x = candidate.x;
      double candidate_tl_y = candidate.y;
      double candidate_br_x = candidate.x + candidate.width;
      double candidate_br_y = candidate.y + candidate.height;

      double tl_x = std::max(bbox_tl_x, candidate_tl_x);
      double tl_y = std::max(bbox_tl_y, candidate_tl_y);
      double br_x = std::min(bbox_br_x, candidate_br_x);
      double br_y = std::min(bbox_br_y, candidate_br_y);

      double width = std::max(0.0, br_x - tl_x);
      double height = std::max(0.0, br_y - tl_y);

      double area_intersection = width * height;
      double area_bbox = bbox.width * bbox.height;
      double area_candidate = candidate.width * candidate.height;

      return area_intersection / (area_bbox + area_candidate - area_intersection);
}

std::vector<double> IoU::calculateIoU(const BoundingBox &bbox, const std::vector<BoundingBox> &candidates)
{
      std::vector<double> iouScores;
      for (const BoundingBox &candidate : candidates)
      {
            double iou = calculateIntersectionOverUnion(bbox, candidate);
            iouScores.push_back(iou);
      }
      return iouScores;
}

std::vector<std::vector<double>> IoU::calculateIoUCost(
    const std::vector<Track> &tracks,
    const std::vector<Detection> &detections,
    const std::vector<int> &trackIndices,
    const std::vector<int> &detectionIndices)
{
      std::vector<std::vector<double>> costMatrix(trackIndices.size(), std::vector<double>(detectionIndices.size(), 0.0));
      for (int row = 0; row < trackIndices.size(); ++row)
      {
            int trackIdx = trackIndices[row];
            if (tracks[trackIdx].time_since_update > 1)
            {
                  std::fill(costMatrix[row].begin(), costMatrix[row].end(), INFTY_COST);
                  continue;
            }

            BoundingBox bbox = tracks[trackIdx].to_tlwh();
            std::vector<BoundingBox> candidates;
            for (int i : detectionIndices)
            {
                  candidates.push_back(detections[i]);
            }

            std::vector<double> iouScores = calculateIoU(bbox, candidates);
            for (int col = 0; col < detectionIndices.size(); ++col)
            {
                  costMatrix[row][col] = 1.0 - iouScores[col];
            }
      }

      return costMatrix;
}
