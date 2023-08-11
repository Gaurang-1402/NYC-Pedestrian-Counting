#include "preprocessing.hpp"

std::vector<int> non_max_suppression(const std::vector<cv::Rect> &boxes, const std::vector<int> &classes, float max_bbox_overlap, const std::vector<float> &scores)
{
      std::vector<int> pick;

      if (boxes.empty())
      {
            return pick;
      }

      std::vector<cv::Rect> boxesCopy = boxes;

      if (!scores.empty())
      {
            std::vector<int> idxs(scores.size());
            std::iota(idxs.begin(), idxs.end(), 0);
            std::sort(idxs.begin(), idxs.end(), [&scores](int i1, int i2)
                      { return scores[i1] < scores[i2]; });

            while (!idxs.empty())
            {
                  int last = idxs.size() - 1;
                  int i = idxs[last];
                  pick.push_back(i);

                  std::vector<int> rest(idxs.begin(), idxs.begin() + last);
                  std::vector<int> toDelete;

                  for (int j : rest)
                  {
                        int xx1 = std::max(boxesCopy[i].x, boxesCopy[j].x);
                        int yy1 = std::max(boxesCopy[i].y, boxesCopy[j].y);
                        int xx2 = std::min(boxesCopy[i].x + boxesCopy[i].width, boxesCopy[j].x + boxesCopy[j].width);
                        int yy2 = std::min(boxesCopy[i].y + boxesCopy[i].height, boxesCopy[j].y + boxesCopy[j].height);

                        int w = std::max(0, xx2 - xx1 + 1);
                        int h = std::max(0, yy2 - yy1 + 1);

                        float overlap = static_cast<float>(w * h) / ((boxesCopy[i].width + 1) * (boxesCopy[i].height + 1));

                        if (overlap > max_bbox_overlap)
                        {
                              toDelete.push_back(j);
                        }
                  }

                  for (int j : toDelete)
                  {
                        idxs.erase(std::remove(idxs.begin(), idxs.end(), j), idxs.end());
                  }
            }
      }
      else
      {
            std::vector<int> idxs(boxesCopy.size());
            std::iota(idxs.begin(), idxs.end(), 0);
            std::sort(idxs.begin(), idxs.end(), [&boxesCopy](int i1, int i2)
                      { return boxesCopy[i1].y < boxesCopy[i2].y; });

            while (!idxs.empty())
            {
                  int last = idxs.size() - 1;
                  int i = idxs[last];
                  pick.push_back(i);

                  std::vector<int> rest(idxs.begin(), idxs.begin() + last);
                  std::vector<int> toDelete;

                  for (int j : rest)
                  {
                        int xx1 = std::max(boxesCopy[i].x, boxesCopy[j].x);
                        int yy1 = std::max(boxesCopy[i].y, boxesCopy[j].y);
                        int xx2 = std::min(boxesCopy[i].x + boxesCopy[i].width, boxesCopy[j].x + boxesCopy[j].width);
                        int yy2 = std::min(boxesCopy[i].y + boxesCopy[i].height, boxesCopy[j].y + boxesCopy[j].height);

                        int w = std::max(0, xx2 - xx1 + 1);
                        int h = std::max(0, yy2 - yy1 + 1);

                        float overlap = static_cast<float>(w * h) / ((boxesCopy[i].width + 1) * (boxesCopy[i].height + 1));

                        if (overlap > max_bbox_overlap)
                        {
                              toDelete.push_back(j);
                        }
                  }

                  for (int j : toDelete)
                  {
                        idxs.erase(std::remove(idxs.begin(), idxs.end(), j), idxs.end());
                  }
            }
      }

      return pick;
}
