#include "track.hpp"

Track::Track(const cv::Mat &mean, const cv::Mat &covariance, int track_id, int n_init, int max_age,
             const cv::Mat &feature, const std::string &class_name)
    : mean_(mean),
      covariance_(covariance),
      track_id_(track_id),
      hits_(1),
      age_(1),
      time_since_update_(0),
      state_(TrackState::Tentative),
      n_init_(n_init),
      max_age_(max_age),
      class_name_(class_name)
{
      if (!feature.empty())
      {
            features_.push_back(feature);
      }
}

cv::Rect2f Track::to_tlwh() const
{
      cv::Rect2f ret(mean_.at<float>(0), mean_.at<float>(1), mean_.at<float>(2), mean_.at<float>(3));
      ret.width *= ret.height;
      ret.x -= ret.width / 2;
      ret.y -= ret.height / 2;
      return ret;
}

cv::Rect2f Track::to_tlbr() const
{
      cv::Rect2f ret = to_tlwh();
      ret.width += ret.x;
      ret.height += ret.y;
      return ret;
}

std::string Track::get_class() const
{
      return class_name_;
}

void Track::predict(cv::KalmanFilter &kf)
{
      cv::Mat prediction = kf.predict(mean_, covariance_);
      mean_ = prediction.colRange(0, 4);
      covariance_ = prediction.colRange(4, 10);
      age_++;
      time_since_update_++;
}

void Track::update(cv::KalmanFilter &kf, const cv::Mat &detection)
{
      cv::Mat measurement = cv::Mat(1, 4, CV_32F);
      measurement.at<float>(0) = detection.at<float>(0);
      measurement.at<float>(1) = detection.at<float>(1);
      measurement.at<float>(2) = detection.at<float>(2);
      measurement.at<float>(3) = detection.at<float>(3);

      cv::Mat update = kf.update(mean_, covariance_, measurement);
      mean_ = update.colRange(0, 4);
      covariance_ = update.colRange(4, 10);

      features_.push_back(detection);

      hits_++;
      time_since_update_ = 0;

      if (state_ == TrackState::Tentative && hits_ >= n_init_)
      {
            state_ = TrackState::Confirmed;
      }
}

void Track::mark_missed()
{
      if (state_ == TrackState::Tentative)
      {
            state_ = TrackState::Deleted;
      }
      else if (time_since_update_ > max_age_)
      {
            state_ = TrackState::Deleted;
      }
}

bool Track::is_tentative() const
{
      return state_ == TrackState::Tentative;
}

bool Track::is_confirmed() const
{
      return state_ == TrackState::Confirmed;
}

bool Track::is_deleted() const
{
      return state_ == TrackState::Deleted;
}
