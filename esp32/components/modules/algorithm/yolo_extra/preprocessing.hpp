#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP

#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>

std::vector<int> non_max_suppression(const std::vector<cv::Rect> &boxes, const std::vector<int> &classes, float max_bbox_overlap, const std::vector<float> &scores = {});

#endif // PREPROCESSING_HPP
