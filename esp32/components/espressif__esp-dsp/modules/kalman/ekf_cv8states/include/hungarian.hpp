#ifndef HUNGARIAN_HPP
#define HUNGARIAN_HPP

#include <vector>
#include <algorithm>
#include <limits>
#include <utility>

using namespace std;

class LinearSumAssignment
{
      vector<vector<double>> costMatrix;
      vector<int> worker, job;

      void augment(int n, int max_match, vector<int> &xy, vector<int> &yx, vector<bool> &S, vector<bool> &T, vector<int> &slackx, vector<double> &slack, vector<int> &prev)
      {
            if (max_match == n)
                  return;
            int x, y, root;
            vector<int> q(n);
            int wr = 0, rd = 0;
            vector<int> set(n, -1);
            S.assign(n, false);
            T.assign(n, false);
            prev.assign(n, -1);

            for (x = 0; x < n; x++)
            {
                  if (xy[x] == -1)
                  {
                        q[wr++] = root = x;
                        prev[x] = -2;
                        S[x] = true;
                        break;
                  }
            }

            for (y = 0; y < n; y++)
            {
                  slack[y] = costMatrix[root][y] - worker[root] - job[y];
                  slackx[y] = root;
            }

            while (true)
            {
                  while (rd < wr)
                  {
                        x = q[rd++];
                        for (y = 0; y < n; y++)
                        {
                              if (costMatrix[x][y] == worker[x] + job[y] && !T[y])
                              {
                                    if (yx[y] == -1)
                                          break;
                                    T[y] = true;
                                    q[wr++] = yx[y];
                                    add_to_tree(yx[y], x, S, T, slack, slackx, set, prev, q, wr, worker, job, costMatrix);
                              }
                        }
                        if (y < n)
                              break;
                  }
                  if (y < n)
                        break;

                  update_labels(n, S, T, slack, slackx, worker, job);
                  wr = rd = 0;
                  for (y = 0; y < n; y++)
                  {
                        if (!T[y] && slack[y] == 0)
                        {
                              if (yx[y] == -1)
                              {
                                    x = slackx[y];
                                    break;
                              }
                              else
                              {
                                    T[y] = true;
                                    if (!S[yx[y]])
                                    {
                                          q[wr++] = yx[y];
                                          add_to_tree(yx[y], slackx[y], S, T, slack, slackx, set, prev, q, wr, worker, job, costMatrix);
                                    }
                              }
                        }
                  }
                  if (y < n)
                        break;
            }

            if (y < n)
            {
                  max_match++;
                  for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty)
                  {
                        ty = xy[cx];
                        yx[cy] = cx;
                        xy[cx] = cy;
                  }
                  augment(n, max_match, xy, yx, S, T, slackx, slack, prev);
            }
      }

      void add_to_tree(int x, int prevx, vector<bool> &S, vector<bool> &T, vector<double> &slack, vector<int> &slackx, vector<int> &set, vector<int> &prev, vector<int> &q, int &wr, vector<int> &worker, vector<int> &job, vector<vector<double>> &costMatrix)
      {
            S[x] = true;
            prev[x] = prevx;
            for (int y = 0; y < slack.size(); y++)
            {
                  if (costMatrix[x][y] - worker[x] - job[y] < slack[y])
                  {
                        slack[y] = costMatrix[x][y] - worker[x] - job[y];
                        slackx[y] = x;
                  }
            }
      }

      void update_labels(int n, vector<bool> &S, vector<bool> &T, vector<double> &slack, vector<int> &slackx, vector<int> &worker, vector<int> &job)
      {
            double delta = numeric_limits<double>::max();
            for (int y = 0; y < n; y++)
                  if (!T[y])
                        delta = min(delta, slack[y]);
            for (int x = 0; x < n; x++)
                  if (S[x])
                        worker[x] -= delta;
            for (int y = 0; y < n; y++)
                  if (T[y])
                        job[y] += delta;
                  else
                        slack[y] -= delta;
      }

public:
      pair<vector<int>, vector<int>> solve(vector<vector<double>> &costMatrix)
      {
            int n = costMatrix.size();
            this->costMatrix = costMatrix;
            worker.assign(n, 0);
            job.assign(n, 0);
            vector<int> xy(n, -1);
            vector<int> yx(n, -1);
            vector<bool> S, T;
            vector<int> slackx(n);
            vector<double> slack(n);
            vector<int> prev(n);

            augment(n, 0, xy, yx, S, T, slackx, slack, prev);

            return make_pair(xy, yx);
      }
};

#endif // HUNGARIAN_HPP