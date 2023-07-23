
#include "tracker.h"

struct Tracker *tracker_new(void)
{
      struct Tracker *tracker = malloc(sizeof(struct Tracker));
      tracker->kf = kalman_filter_new();
      tracker->tracks = NULL;
      tracker->next_id = 1;
      return tracker;
}

void tracker_free(struct Tracker *tracker)
{
      kalman_filter_free(tracker->kf);
      for (int i = 0; tracker->tracks != NULL && i < tracker->tracks->len; i++)
      {
            track_free(tracker->tracks[i]);
      }
      free(tracker->tracks);
      free(tracker);
}

void tracker_predict(struct Tracker *tracker)
{
      for (int i = 0; tracker->tracks != NULL && i < tracker->tracks->len; i++)
      {
            track_predict(tracker->tracks[i], tracker->kf);
      }
}

void tracker_update(struct Tracker *tracker, struct Detection *detections)
{
      int n_detections = detections->len;
      int n_tracks = tracker->tracks->len;
      int *matches = malloc(sizeof(int) * n_detections);
      int *unmatched_tracks = malloc(sizeof(int) * n_tracks);
      int *unmatched_detections = malloc(sizeof(int) * n_detections);

      linear_assignment_matching_cascade(
          gated_metric, tracker->tracks, detections, matches, unmatched_tracks,
          unmatched_detections);

      for (int i = 0; i < n_detections; i++)
      {
            if (matches[i] != -1)
            {
                  track_update(tracker->tracks[matches[i]], tracker->kf, detections[i]);
            }
            else
            {
                  track_initiate(tracker, detections[i]);
            }
      }

      for (int i = 0; i < n_tracks; i++)
      {
            if (unmatched_tracks[i] != -1)
            {
                  track_mark_missed(tracker->tracks[i]);
            }
      }

      free(matches);
      free(unmatched_tracks);
      free(unmatched_detections);
}

int main(void)
{
      struct Tracker *tracker = tracker_new();
      struct Detection *detections = detection_new_array(3);

      tracker_predict(tracker);
      tracker_update(tracker, detections);

      tracker_free(tracker);
      detection_free_array(detections);

      return 0;
}