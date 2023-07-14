#ifndef IOU_MATCHING_HPP
#define IOU_MATCHING_HPP

#include <vector>
#include "linear_assignment.hpp"
#include "track.hpp"
#define INFTY_COST 1e+5

struct BoundingBox
{
      double x;
      double y;
      double width;
      double height;
};

class IoU
{
public:
      static double calculateIntersectionOverUnion(const BoundingBox &bbox, const BoundingBox &candidate);
      static std::vector<double> calculateIoU(const BoundingBox &bbox, const std::vector<BoundingBox> &candidates);
      static std::vector<std::vector<double>> calculateIoUCost(
          const std::vector<Track> &tracks,
          const std::vector<Detection> &detections,
          const std::vector<int> &trackIndices = std::vector<int>(),
          const std::vector<int> &detectionIndices = std::vector<int>());
};

#endif // IOU_MATCHING_HPP
