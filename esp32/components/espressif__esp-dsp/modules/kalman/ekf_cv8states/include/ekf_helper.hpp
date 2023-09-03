#ifndef EKF_HELPER_HPP
#define EKF_HELPER_HPP

#include <ekf.h>
#include "tracking_utils.hpp"

class KalmanBoxTracker : public ekf
{
public:
      static int count;
      int id;
      int time_since_update;
      int hits;
      int hit_streak;
      int age;
      int objclass;
      vector<vector<double>> history;
      dspm::Mat R;

      KalmanBoxTracker(vector<double> bbox) : ekf(7, 4)
      {

            auto converted_bbox = convert_bbox_to_z(bbox);
            for (int i = 0; i < 4; i++)
                  X(0, i) = converted_bbox[i];

            time_since_update = 0;
            id = KalmanBoxTracker::count;
            KalmanBoxTracker::count += 1;
            hits = 0;
            hit_streak = 0;
            age = 0;
            objclass = 0;
      }

      KalmanBoxTracker(const KalmanBoxTracker &x_) : ekf(7, 4)
      {
            this->id = x_.id;
            this->time_since_update = x_.time_since_update;
            this->hits = x_.hits;
            this->hit_streak = x_.hit_streak;
            this->age = x_.age;
            this->objclass = x_.objclass;
            this->history = x_.history;

            this->R = dspm::Mat(x_.R);

            this->X = dspm::Mat(x_.X);
            this->F = dspm::Mat(x_.F);
            this->G = dspm::Mat(x_.G);

            this->P = dspm::Mat(x_.P);
            this->Q = dspm::Mat(x_.Q);

            this->NUMX = x_.NUMX;
            this->NUMW = x_.NUMW;
            this->HP = new float[this->NUMX];
            this->Km = new float[this->NUMX];
            for (size_t i = 0; i < this->NUMX; i++)
            {
                  this->HP[i] = x_.HP[i];
                  this->Km[i] = x_.Km[i];
            }
      }

      ~KalmanBoxTracker()
      {
      }

      KalmanBoxTracker &
      operator=(const KalmanBoxTracker &x_)
      {
            this->id = x_.id;
            this->time_since_update = x_.time_since_update;
            this->hits = x_.hits;
            this->hit_streak = x_.hit_streak;
            this->age = x_.age;
            this->objclass = x_.objclass;
            this->history = x_.history;
            // Delete our old matrices
            // printf("Deleting old matrices\n");

            // printf("Deleted old matrices\n");

            this->R = dspm::Mat(x_.R);

            this->X = dspm::Mat(x_.X);
            this->F = dspm::Mat(x_.F);
            this->G = dspm::Mat(x_.G);

            this->P = dspm::Mat(x_.P);
            this->Q = dspm::Mat(x_.Q);

            this->NUMX = x_.NUMX;
            this->NUMW = x_.NUMW;
            // printf("Copied new matrices\n");
            for (size_t i = 0; i < this->NUMX; i++)
            {
                  this->HP[i] = x_.HP[i];
                  this->Km[i] = x_.Km[i];
            }
            // printf("Copied new matrices_2\n");

            return *this;
      }

      virtual void LinearizeFG(dspm::Mat &x, float *u)
      {

            this->F *= 0; // Initialize F and G matrixes.
            this->G *= 0;

            // dqdot / dq - skey matrix
            // F.Copy(0.5 * ekf::SkewSym4x4(w), 0, 0);
            for (int i = 0; i < 7; i++)
                  F(i, i) = 1.0;

            for (int i = 4; i < 7; i++)
                  F(i, i - 4) = 1.0;

            // dqdot/dvector
            // dspm::Mat dq = -0.5 * qProduct(x.data);
            // dspm::Mat dq_q = dq.Get(0, 4, 1, 3);

            // // dqdot / dnw
            // G.Copy(dq_q, 0, 0);
            // dqdot / dwbias
            // F.Copy(dq_q, 0, 4);

            R = dspm::Mat::eye(4);

            R(0, 0) *= 10.0;
            R(1, 1) *= 10.0;

            // multiply all P value by 10
            P *= 10;

            Q(3, 3) *= 0.01;

            for (int i = 0; i < 4; ++i)
            {
                  Q(i, i) *= 0.01;
            }
      }

      virtual void Init(){};

      virtual dspm::Mat StateXdot(dspm::Mat &x, float *u)
      {
            return dspm::Mat();
      };

      void update(const vector<double> &bbox, float dt = 1.0)
      {
            time_since_update = 0;
            history.clear();
            hits += 1;
            hit_streak += 1;

            vector<double> converted_bbox = convert_bbox_to_z(bbox);
            float u[4];
            for (int i = 0; i < 4; i++)
                  u[i] = converted_bbox[i];

            Process(u, dt);
      }

      vector<double> predict(float dt = 1.0)
      {
            if (X(1, 6) + X(1, 2) <= 0)
                  X(0, 6) *= 0.0;
            float u[4] = {0, 0, 0, 0};
            Process(u, dt);
            age += 1;
            if (time_since_update > 0)
                  hit_streak = 0;
            time_since_update += 1;

            vector<double> tmp_x(4);
            for (int i = 0; i < 4; ++i)
            {
                  tmp_x[i] = X(0, i);
            }
            history.push_back(convert_x_to_bbox(tmp_x));
            return history.back();
      }

      vector<double> get_state()
      {
            vector<double> tmp_x(4);
            for (int i = 0; i < 4; ++i)
            {
                  tmp_x[i] = X(0, i);
            }

            // Print the current state
            printf("Current state: [");
            for (int i = 0; i < 2; ++i)
            {
                  printf("[ ");
                  for (int j = 0; j < 4; ++j)
                  {
                        printf("%f ", X(i, j));
                  }
                  printf("] ");
            }
            printf("]\n");

            return convert_x_to_bbox(tmp_x);
      }
};

int KalmanBoxTracker::count = 0;

#endif // EKF_HELPER_HPP