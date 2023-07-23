#ifndef TRACK_HPP
#define TRACK_HPP

#include <vector>
#include <opencv2/core.hpp>
#include <functional>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>

enum class TrackState
{
      Tentative = 1,
      Confirmed = 2,
      Deleted = 3
};

class Track
{
public:
      Track(const cv::Mat &mean, const cv::Mat &covariance, int track_id, int n_init, int max_age,
            const cv::Mat &feature = cv::Mat(), const std::string &class_name = nullptr);

      cv::Rect2f to_tlwh() const;
      cv::Rect2f to_tlbr() const;
      std::string get_class() const;

      void predict(cv::KalmanFilter &kf);
      void update(cv::KalmanFilter &kf, const cv::Mat &detection);

      void mark_missed();
      bool is_tentative() const;
      bool is_confirmed() const;
      bool is_deleted() const;

private:
      Eigen::VectorXd mean;
      Eigen::MatrixXd covariance;
      int track_id_;
      int hits_;
      int age_;
      int time_since_update_;
      TrackState state_;
      std::vector<cv::Mat> features_;
      int n_init_;
      int max_age_;
      std::string class_name_;
};

#endif // TRACK_HPP
