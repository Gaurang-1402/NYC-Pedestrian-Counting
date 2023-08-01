#ifndef EKF_HELPER_HPP
#define EKF_HELPER_HPP

#include <ekf.h>
#include "tracking_utils.hpp"

class KalmanBoxTracker : private ekf
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

            // kf.F = {{1, 0, 0, 0, 1, 0, 0},
            //         {0, 1, 0, 0, 0, 1, 0},
            //         {0, 0, 1, 0, 0, 0, 1},
            //         {0, 0, 0, 1, 0, 0, 0},
            //         {0, 0, 0, 0, 1, 0, 0},
            //         {0, 0, 0, 0, 0, 1, 0},
            //         {0, 0, 0, 0, 0, 0, 1}};

            /*

            x: state
            P: uncertainty covariance
            Q: process uncertainty
            B: control transition matrix
            F: state transition matrix
            H: Measurement function
            R: state uncertainty
            M: process-measurement cross correlation

            K: kalmna gain

            S: system uncertainty
            */
            // kf.H = {{1, 0, 0, 0, 0, 0, 0},
            //         {0, 1, 0, 0, 0, 0, 0},
            //         {0, 0, 1, 0, 0, 0, 0},
            //         {0, 0, 0, 1, 0, 0, 0}};

            // TODO: You'll need to implement the equivalent operations for the R, P and Q matrices
            // The below lines are placeholders for reference
            // kf.R[2:,2:] *= 10.
            // kf.P[4:,4:] *= 1000.
            // kf.P *= 10.
            // kf.Q[-1,-1] *= 0.01
            // kf.Q[4:,4:] *= 0.01

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

      void LinearizeFG(dspm::Mat &x, float *u)
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
            dspm::Mat dq = -0.5 * qProduct(x.data);
            dspm::Mat dq_q = dq.Get(0, 4, 1, 3);

            // dqdot / dnw
            G.Copy(dq_q, 0, 0);
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

      // dspm::Mat StateXdot(dspm::Mat &x, float *u)
      // {

      //       dspm::Mat xdot(7, 1);
      //       dspm::Mat w(3, 1);
      //       dspm::Mat wbias(3, 1);

      //       for (int i = 0; i < 3; i++)
      //       {
      //             w(i, 0) = u[i];
      //             wbias(i, 0) = u[i + 3];
      //       }

      //       dspm::Mat q = x.Get(0, 4, 1, 3);
      //       dspm::Mat qdot = 0.5 * qProduct(w);
      //       dspm::Mat qdot_q = qdot.Get(0, 4, 1, 3);

      //       xdot.Copy(qdot_q, 0, 0);
      //       xdot.Copy(wbias, 4, 0);

      //       return xdot;
      // }

      void update(vector<double> bbox)
      {
            time_since_update = 0;
            history.clear();
            hits += 1;
            hit_streak += 1;

            auto converted_bbox = convert_bbox_to_z(bbox);
            float u[4];
            for (int i = 0; i < 4; i++)
                  u[i] = converted_bbox[i];

            Process(u, 1);
      }

      vector<double> predict()
      {
            if (X(1, 6) + X(1, 2) <= 0)
                  X(0, 6) *= 0.0;
            float u[4] = {0, 0, 0, 0};
            Process(u, 1);
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

            return convert_x_to_bbox(tmp_x);
      }
};

int KalmanBoxTracker::count = 0;

#endif // EKF_HELPER_HPP