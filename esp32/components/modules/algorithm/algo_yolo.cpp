#include <math.h>
#include <stdint.h>
#include <forward_list>

#include "algo_yolo.hpp"
#include "yolo_model_data.h"

#include "fb_gfx.h"
#include "isp.h"
#include "base64.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include <iostream>
#include <algorithm>
using std::min;
using std::max;

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include <vector>

#include "dsp_platform.h"
#include "esp_dsp.h"
#include "ekf.h"

// TODO fix imports
#include "Hungarian.h"


static const char *TAG = "yolo";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gEvent = true;
static bool gReturnFB = true;
static bool debug_mode = false;

#define CONFIDENCE 40
#define IOU 30

const uint16_t box_color[] = {0x1FE0, 0x07E0, 0x001F, 0xF800, 0xF81F, 0xFFE0};


/* ================================= SORT TRACKER ==================================== */
// TODO remove after you figure out kalman filter
class KalmanFilter {
public:
    std::vector<double> state;
    std::vector<std::vector<double>> covariance;

    KalmanFilter() {
        // Initialization of the state and covariance goes here
    }

    void initialize(std::vector<double>& initialState) {
        state = initialState;
        // More initialization code might go here
    }

    std::vector<double> predict() {
        // Prediction code goes here. For now, just return the current state
        return state;
    }

    std::vector<double> update(std::vector<double>& measurement) {
        // Update code goes here. For now, just set the state to the measurement and return it
        state = measurement;
        return state;
    }
};


/*

his refactoring assumes you use the Process method for prediction and the Update method for updating the filter state based on the measurements. The usage of the ekf's Update method may be different depending on your specific implementation. The H matrix, measurement, expected values, and R noise values would need to be appropriately set in your code before calling the update method.

The new TrackedObject class is initialized with the dimensions of the state and the noise vectors and the initial state. The predict method now also takes a control input u and the time step dt as parameters, which are required by the ekf's Process method. The update method takes the same parameters as the ekf's Update method.

Additionally, please remember to deal with memory management appropriately as you are using raw pointers for the measurements, expected values, and noise in your ekf's Update method.

*/

// class TrackedObject {
// public:
//     ekf filter;  // The extended Kalman filter that will track this object
//     dspm::Mat state;  // The state of the object

//     TrackedObject(int state_dim, int noise_dim, dspm::Mat initialState) : state(initialState), filter(state_dim, noise_dim) {
//         filter.X = state;  // Initialize the state of the filter
//     }

//     void predict(float *u, float dt) {
//         filter.Process(u, dt);  // Predict the next state using the EKF
//         state = filter.X; // Update object state
//     }

//     void update(dspm::Mat &H, float *measurement, float *expected, float *R) {
//         filter.Update(H, measurement, expected, R);  // Update the state based on the new measurement
//         state = filter.X; // Update object state
//     }
// };



class TrackedObject {
public:
    KalmanFilter kf;  // The Kalman filter that will track this object
    std::vector<double> state;  // The state of the object

    TrackedObject(std::vector<double> initialState) : state(initialState) {
        kf = KalmanFilter();  // Initialize a new Kalman filter for this object
        kf.initialize(state);  // Initialize the Kalman filter with the first state
    }

    void predict() {
        state = kf.predict();  // Predict the next state using the Kalman filter
    }

    void update(std::vector<double> measurement) {
        state = kf.update(measurement);  // Update the state based on the new measurement
    }
};

std::vector<std::vector<double>> getMeasurements(std::forward_list<yolo_t> yolo_list) {
    std::vector<std::vector<double>> measurements;
    for (const auto& yolo : yolo_list) {
        std::vector<double> measurement;
        measurement.push_back(static_cast<double>(yolo.x));
        measurement.push_back(static_cast<double>(yolo.y));
        measurement.push_back(static_cast<double>(yolo.w));
        measurement.push_back(static_cast<double>(yolo.h));
        measurements.push_back(measurement);
    }
    return measurements;
}

double calculateDistanceEucledian(std::vector<double>& state, std::vector<double>& measurement) {
    double dx = state[0] - measurement[0];
    double dy = state[1] - measurement[1];
    return std::sqrt(dx * dx + dy * dy);
}

double calculateDistanceIoU(std::vector<double>& state, std::vector<double>& measurement) {
    // Calculate overlap
    double x_overlap = std::max(0.0, std::min(state[0] + state[2]/2, measurement[0] + measurement[2]/2) - std::max(state[0] - state[2]/2, measurement[0] - measurement[2]/2));
    double y_overlap = std::max(0.0, std::min(state[1] + state[3]/2, measurement[1] + measurement[3]/2) - std::max(state[1] - state[3]/2, measurement[1] - measurement[3]/2));
    double intersection = x_overlap * y_overlap;

    // Calculate union
    double stateArea = state[2] * state[3];
    double measurementArea = measurement[2] * measurement[3];
    double union_ = stateArea + measurementArea - intersection;

    // Calculate IoU
    double IoU = intersection / union_;

    // In this case, a higher overlap means a smaller "distance".
    // So we return the inverse of the IoU. You might need to adjust this to fit your needs.
    return 1 - IoU;
}



std::vector<std::vector<double>> computeCostMatrix(std::vector<TrackedObject>& objects, std::vector<std::vector<double>>& measurements) {
    std::vector<std::vector<double>> costMatrix(objects.size(), std::vector<double>(measurements.size()));

    for (int i = 0; i < objects.size(); i++) {
        for (int j = 0; j < measurements.size(); j++) {
            costMatrix[i][j] = calculateDistanceEucledian(objects[i].state, measurements[j]);  // calculateDistance is a function you need to implement
        }
    }

    // for (int i = 0; i < objects.size(); i++) {
    //     for (int j = 0; j < measurements.size(); j++) {
    //         costMatrix[i][j] = calculateDistanceIoU(objects[i].state, measurements[j]);  // calculateDistance is a function you need to implement
    //     }
    // }

    return costMatrix;
}


// // Global counter
// int current_id = 0;


// // TODO come up with a clever way for unique ID
// // Function to generate a new ID
// int get_new_id() {
//     return current_id++;
// }





/* ================================ LINE CROSSING METHOD ====================================== */

// Start by defining points and lines to represent the counting lines


struct Point {
    int x, y;
    Point(int _x, int _y) : x(_x), y(_y) {}
};

struct Line {
    Point p1, p2;
    Line(Point _p1, Point _p2) : p1(_p1), p2(_p2) {}
};


// Global pedestrian count for each line
int pedCountHorizontal = 0;
int pedCountVertical = 0;
int pedCountDiagonal = 0;

// Function to find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
       return true;

    return false;
}


// Function that returns true if line segment 'p1q1' and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

// bool crosses_line(Centroid last_centroid, Centroid centroid, Line line) {
//     // Check if the line segment from last_centroid to centroid crosses the line.
//     // Return true if it does, false otherwise.


//     Point p1(last_centroid.x, last_centroid.y);
//     Point q1(centroid.x, centroid.y);

//     return doIntersect(p1, q1, line.p1, line.p2);
// }

/* ====================================================================== */



/* ============================== CENTROID TRACKER ======================================== */


// void register_object(Centroid centroid) {
//     TrackedObject new_object;
//     new_object.id = get_new_id();
//     new_object.centroid = centroid;
//     new_object.last_centroid = centroid;
//     new_object.disappeared = 0;
//     objects.push_back(new_object);
// }

// void deregister_object(int index) {
//     objects.erase(objects.begin() + index);
// }
// // Maximum distance between an object's old centroid and a new centroid for them
// // to be considered the same object.
// const int max_distance = 50;


// void update(std::vector<Centroid> new_centroids, Line horizontalLine, Line verticalLine, Line diagonalLine) {
//     if (new_centroids.empty()) {
//         for (auto& object : objects) {
//             object.disappeared++;

//             if (object.disappeared > 10) {
//                 deregister_object(object.id);
//             }
//         }
//         return;
//     }

//     if (objects.empty()) {
//         for (const auto& centroid : new_centroids) {
//             register_object(centroid);
//         }
//         std::cout << "\033[1;32mRegistered " << new_centroids.size() << " new objects.\033[0m\n";
//         return;
//     }

//     std::vector<std::vector<int>> distances(objects.size(), std::vector<int>(new_centroids.size()));
//     for (int i = 0; i < objects.size(); ++i) {
//         for (int j = 0; j < new_centroids.size(); ++j) {
//             distances[i][j] = std::abs(objects[i].centroid.x - new_centroids[j].x) +
//                               std::abs(objects[i].centroid.y - new_centroids[j].y);
//         }
//     }

//     for (int i = 0; i < objects.size(); ++i) {
//         int min_distance = max_distance + 1;
//         int closest_j = -1;
//         for (int j = 0; j < new_centroids.size(); ++j) {
//             if (distances[i][j] < min_distance) {
//                 min_distance = distances[i][j];
//                 closest_j = j;
//             }
//         }

//         if (closest_j != -1) {
//             // Before updating the current centroid, store its value in last_centroid
//             objects[i].last_centroid = objects[i].centroid;

//             // Now you can update the current centroid
//             objects[i].centroid = new_centroids[closest_j];
//             objects[i].disappeared = 0;

//             // Check if the object crossed the lines.
//             if (crosses_line(objects[i].last_centroid, objects[i].centroid, horizontalLine)) {
//                 pedCountHorizontal++;
//             }
//             if (crosses_line(objects[i].last_centroid, objects[i].centroid, verticalLine)) {
//                 pedCountVertical++;
//             }
//             if (crosses_line(objects[i].last_centroid, objects[i].centroid, diagonalLine)) {
//                 pedCountDiagonal++;
//             }


//             new_centroids[closest_j].x = new_centroids[closest_j].y = -1;
//             std::cout << "\033[1;34mObject " << objects[i].id << " updated with new centroid.\033[0m\n";
//         } else {
//             objects[i].disappeared++;
//             std::cout << "\033[1;31mObject " << objects[i].id << " has disappeared.\033[0m\n";
//         }
//     }

//     for (const auto& centroid : new_centroids) {
//         if (centroid.x != -1 && centroid.y != -1) {
//             register_object(centroid);
//             std::cout << "\033[1;33mRegistered new object with centroid.\033[0m\n";
//         }
//     }
// }




/* ====================================================================== */



std::forward_list<yolo_t> nms_get_obeject_topn(int8_t *dataset, uint16_t top_n, uint8_t threshold, uint8_t nms, uint16_t width, uint16_t height, int num_record, int8_t num_class, float scale, int zero_point);



// Globals, used for compatibility with Arduino-style sketches.
namespace
{
    const tflite::Model *model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    TfLiteTensor *input = nullptr;
    static std::forward_list<yolo_t> _yolo_list;

    // In order to use optimized tensorflow lite kernels, a signed int8_t quantized
    // model is preferred over the legacy unsigned model format. This means that
    // throughout this project, input images must be converted from unisgned to
    // signed format. The easiest and quickest way to convert from unsigned to
    // signed 8-bit integers is to subtract 128 from the unsigned value to get a
    // signed value.

#ifdef CONFIG_IDF_TARGET_ESP32S3
    constexpr int scratchBufSize = 1024 * 1024;
#else
    constexpr int scratchBufSize = 0;
#endif
    // An area of memory to use for input, output, and intermediate arrays.
    constexpr int kTensorArenaSize = 256 * 1024 + scratchBufSize;
    static uint8_t *tensor_arena; //[kTensorArenaSize]; // Maybe we should move this to external
} //

static void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;

    uint16_t h = input->dims->data[1];
    uint16_t w = input->dims->data[2];
    uint16_t c = input->dims->data[3];

    printf("Format: {\"height\": %d, \"width\": %d, \"channels\": %d, \"model\": \"yolo\"}\r\n", h, w, c);

    // Initialize lines
    Line horizontalLine = Line(Point(0, h / 2), Point(w, h / 2)); // horizontal line
    Line verticalLine = Line(Point(w / 2, 0), Point(w / 2, h)); // vertical line
    Line diagonalLine = Line(Point(0, 0), Point(w, h)); // diagonal line

    // Initialize tracked objects
    // std::vector<TrackedObject> trackedObjects;

    while (true)
    {
        if (gEvent)
        {
            if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
            {


                int dsp_start_time = esp_timer_get_time() / 1000;
                _yolo_list.clear();

                if (c == 1)
                    rgb565_to_gray(input->data.uint8, frame->buf, frame->height, frame->width, h, w, ROTATION_UP);
                else if (c == 3)
                    rgb565_to_rgb888(input->data.uint8, frame->buf, frame->height, frame->width, h, w, ROTATION_UP);

                // for (int i = 0; i < input->bytes; i++)
                // {
                //     frame->buf[i] = input->data.uint8[i];
                // }

                int dsp_end_time = esp_timer_get_time() / 1000;

                if (debug_mode)
                {
                    printf("Begin output\n");
                    printf("Format: {\"height\": %d, \"width\": %d, \"channels\": %d, \"model\": \"yolo\"}\r\n", h, w, c);
                    printf("Framebuffer: ");
                    base64_encode(input->data.uint8, input->bytes, putchar);
                    printf("\r\n");
                }

                for (int i = 0; i < input->bytes; i++)
                {
                    input->data.int8[i] = input->data.uint8[i] - 128;
                }

                // Run the model on this input and make sure it succeeds.
                int start_time = esp_timer_get_time() / 1000;

                if (kTfLiteOk != interpreter->Invoke())
                {
                    MicroPrintf("Invoke failed.");
                }

                int end_time = esp_timer_get_time() / 1000;

                vTaskDelay(10 / portTICK_PERIOD_MS);

                TfLiteTensor *output = interpreter->output(0);

                // Get the results of the inference attempt
                float scale = ((TfLiteAffineQuantization *)(output->quantization.params))->scale->data[0];
                int zero_point = ((TfLiteAffineQuantization *)(output->quantization.params))->zero_point->data[0];

                uint32_t records = output->dims->data[1];
                uint32_t num_class = output->dims->data[2] - OBJECT_T_INDEX;
                int16_t num_element = num_class + OBJECT_T_INDEX;

                // YOLO list
                _yolo_list = nms_get_obeject_topn(output->data.int8, records, CONFIDENCE, IOU, w, h, records, num_class, scale, zero_point);

                // TODO we need to get measurements from the YOLO list
                std::vector<std::vector<double>> measurements = getMeasurements(_yolo_list);

                fb_gfx_drawFastHLine(frame, horizontalLine.p1.x, horizontalLine.p1.y, w, 0xFF0000); // Red
                // Draw vertical line
                fb_gfx_drawFastVLine(frame, verticalLine.p1.x, verticalLine.p1.y, h, 0x00FF00); // Green
                // Draw diagonal line
                fb_gfx_drawLine(frame, diagonalLine.p1.x, diagonalLine.p1.y, diagonalLine.p2.x, diagonalLine.p2.y, 0x0000FF); // Blue


                // Kalman filter predictions
                // for (auto& object : trackedObjects) {
                //     object.predict();
                // }


                // Compute the cost matrix for the Hungarian algorithm
                // std::vector<std::vector<double>> costMatrix = computeCostMatrix(trackedObjects, measurements);

                // // Use the Hungarian algorithm to associate measurements with tracked objects
                // HungarianAlgorithm ha;
                // std::vector<int> assignment;
                // ha.Solve(costMatrix, assignment);

                // for (int i = 0; i < assignment.size(); i++) {
                //     if (assignment[i] != -1) {
                //         trackedObjects[i].update(measurements[assignment[i]]);
                //     } else {
                //         // If no assignment for this object, consider it as lost or create a new one if necessary
                //         // ...
                //     }
                // }

                // for (int i = assignment.size(); i < measurements.size(); i++) {
                //     // If there are any unassigned measurements, create new objects for them
                //     trackedObjects.push_back(TrackedObject(measurements[i]));
                // }

                // Remove objects that have not been updated for a while
                // ...

                // Print pedestrian counts
                std::cout << "\033[1;33mPedestrian Count for Horizontal Line: " << pedCountHorizontal << "\033[0m\n";
                std::cout << "\033[1;33mPedestrian Count for Vertical Line: " << pedCountVertical << "\033[0m\n";
                std::cout << "\033[1;33mPedestrian Count for Diagonal Line: " << pedCountDiagonal << "\033[0m\n"; // Yellow


                printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", (dsp_end_time - dsp_start_time), (end_time - start_time), 0);
                bool found = false;

#if 0
    for (int i = 0; i < records; i++)
    {
        float confidence = float(output->data.int8[i * num_element + OBJECT_C_INDEX] - zero_point) * scale;
        if (confidence > .40)
        {
            int8_t max = -128;
            int target = 0;
            for (int j = 0; j < num_class; j++)
            {
                if (max < output->data.int8[i * num_element + OBJECT_T_INDEX + j])
                {
                    max = output->data.int8[i * num_element + OBJECT_T_INDEX + j];
                    target = j;
                }
            }
            int x = int(float(float(output->data.int8[i * num_element + OBJECT_X_INDEX] - zero_point) * scale) * frame->width);
            int y = int(float(float(output->data.int8[i * num_element + OBJECT_Y_INDEX] - zero_point) * scale) * frame->height);
            int w = int(float(float(output->data.int8[i * num_element + OBJECT_W_INDEX] - zero_point) * scale) * frame->width);
            int h = int(float(float(output->data.int8[i * num_element + OBJECT_H_INDEX] - zero_point) * scale) * frame->height);

            printf("index: %d target: %d max: %d confidence: %d box{x: %d, y: %d, w: %d, h: %d}\n", i, target, max, int((float)confidence * 100), x, y, w, h);
        }
    }
#endif

                if (std::distance(_yolo_list.begin(), _yolo_list.end()) > 0)
                {
                    found = true;
                    printf("    Objects found: %d\n", std::distance(_yolo_list.begin(), _yolo_list.end()));
                    printf("    Objects:\n");
                    printf("    [\n");
                    for (auto &yolo : _yolo_list)
                    {
                        yolo.x = uint16_t(float(yolo.x) / float(w) * float(frame->width));
                        yolo.y = uint16_t(float(yolo.y) / float(h) * float(frame->height));
                        yolo.w = uint16_t(float(yolo.w) / float(w) * float(frame->width));
                        yolo.h = uint16_t(float(yolo.h) / float(h) * float(frame->height));
                        fb_gfx_drawRect(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h / 2, yolo.w, yolo.h, box_color[yolo.target % (sizeof(box_color) / sizeof(box_color[0]))]);
                        // fb_gfx_printf(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h / 2 - 5, 0x1FE0, 0x0000, "%s", g_yolo_model_classes[yolo.target]);
                        printf("        {\"class\": \"%d\", \"x\": %d, \"y\": %d, \"w\": %d, \"h\": %d, \"confidence\": %d},\n", yolo.target, yolo.x, yolo.y, yolo.w, yolo.h, yolo.confidence);
                    }
                    printf("    ]\n");
                }

                if (!found)
                {
                    printf("    No objects found\n");
                }
                if (debug_mode)
                {
                    printf("End output\n");
                }
            }

            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }

            if (xQueueResult)
            {
                xQueueSend(xQueueResult, NULL, portMAX_DELAY);
            }
        }
    }
}

static void task_event_handler(void *arg)
{
    while (true)
    {
        xQueueReceive(xQueueEvent, &(gEvent), portMAX_DELAY);
    }
}

int register_algo_yolo(const QueueHandle_t frame_i,
                       const QueueHandle_t event,
                       const QueueHandle_t result,
                       const QueueHandle_t frame_o,
                       const bool camera_fb_return)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;
    gReturnFB = camera_fb_return;

    // get model (.tflite) from flash
    model = tflite::GetModel(g_yolo_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                    "version %d.",
                    model->version(), TFLITE_SCHEMA_VERSION);
        return -1;
    }

    if (tensor_arena == NULL)
    {
        tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (tensor_arena == NULL)
    {
        printf("Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
        return -1;
    }

    static tflite::MicroMutableOpResolver<18> micro_op_resolver;
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddDepthwiseConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddPad();
    micro_op_resolver.AddPadV2();
    micro_op_resolver.AddAdd();
    micro_op_resolver.AddSub();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddMean();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddConcatenation();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddTranspose();
    micro_op_resolver.AddLogistic();
    micro_op_resolver.AddMul();
    micro_op_resolver.AddSplitV();
    micro_op_resolver.AddStridedSlice();
    micro_op_resolver.AddResizeNearestNeighbor();

    // Build an interpreter to run the model with.
    // NOLINTNEXTLINE(runtime-global-variables)
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        MicroPrintf("AllocateTensors() failed");
        return -1;
    }

    // Get information about the memory area to use for the model's input.
    input = interpreter->input(0);

    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
    if (xQueueEvent)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);

    return 0;
}

#define CLIP(x, y, z) (x < y) ? y : ((x > z) ? z : x)

static bool _object_comparator_reverse(yolo_t &oa, yolo_t &ob)
{
    return oa.confidence < ob.confidence;
}

static bool _object_nms_comparator(yolo_t &oa, yolo_t &ob)
{
    return oa.confidence > ob.confidence;
}

static bool _object_comparator(yolo_t &oa, yolo_t &ob)
{
    return oa.x < ob.x;
}

bool _object_remove(yolo_t &obj)
{
    return (obj.confidence == 0);
}

static uint16_t _overlap(float x1, float w1, float x2, float w2)
{
    uint16_t l1 = x1 - w1 / 2;
    uint16_t l2 = x2 - w2 / 2;
    uint16_t left = l1 > l2 ? l1 : l2;
    uint16_t r1 = x1 + w1 / 2;
    uint16_t r2 = x2 + w2 / 2;
    uint16_t right = r1 < r2 ? r1 : r2;
    return right - left;
}

void _soft_nms_obeject_detection(std::forward_list<yolo_t> &yolo_obj_list, uint8_t nms)
{
    std::forward_list<yolo_t>::iterator max_box_obj;
    yolo_obj_list.sort(_object_comparator);
    for (std::forward_list<yolo_t>::iterator it = yolo_obj_list.begin(); it != yolo_obj_list.end(); ++it)
    {
        uint16_t area = it->w * it->h;
        for (std::forward_list<yolo_t>::iterator itc = std::next(it, 1); itc != yolo_obj_list.end(); ++itc)
        {
            if (itc->confidence == 0)
            {
                continue;
            }
            uint16_t iw = _overlap(itc->x, itc->w, it->x, it->w);
            if (iw > 0)
            {
                uint16_t ih = _overlap(itc->y, itc->h, it->y, it->h);
                if (ih > 0)
                {
                    float ua = float(itc->w * itc->h + area - iw * ih);
                    float ov = iw * ih / ua;
                    if (int(float(ov) * 100) >= nms)
                    {
                        itc->confidence = 0;
                    }
                }
            }
        }
    }
    yolo_obj_list.remove_if(_object_remove);
    return;
}

void _hard_nms_obeject_count(std::forward_list<yolo_t> &yolo_obj_list, uint8_t nms)
{
    std::forward_list<yolo_t>::iterator max_box_obj;
    yolo_obj_list.sort(_object_nms_comparator);
    for (std::forward_list<yolo_t>::iterator it = yolo_obj_list.begin(); it != yolo_obj_list.end(); ++it)
    {
        uint16_t area = it->w * it->h;
        for (std::forward_list<yolo_t>::iterator itc = std::next(it, 1); itc != yolo_obj_list.end(); ++itc)
        {
            if (itc->confidence == 0)
            {
                continue;
            }
            uint16_t iw = _overlap(itc->x, itc->w, it->x, it->w);
            if (iw > 0)
            {
                uint16_t ih = _overlap(itc->y, itc->h, it->y, it->h);
                if (ih > 0)
                {
                    float ua = float(itc->w * itc->h + area - iw * ih);
                    float ov = iw * ih / ua;
                    if (int(float(ov) * 100) >= nms)
                    {
                        itc->confidence = 0;
                    }
                }
            }
        }
    }
    yolo_obj_list.remove_if(_object_remove);

    return;
}

std::forward_list<yolo_t> nms_get_obeject_topn(int8_t *dataset, uint16_t top_n, uint8_t threshold, uint8_t nms, uint16_t width, uint16_t height, int num_record, int8_t num_class, float scale, int zero_point)
{
    bool rescale = scale < 0.1 ? true : false;
    std::forward_list<yolo_t> yolo_obj_list[num_class];
    int16_t num_obj[num_class] = {0};
    int16_t num_element = num_class + OBJECT_T_INDEX;
    for (int i = 0; i < num_record; i++)
    {
        float confidence = float(dataset[i * num_element + OBJECT_C_INDEX] - zero_point) * scale;
        confidence = rescale ? confidence * 100 : confidence;
        if (int(float(confidence)) >= threshold)
        {
            yolo_t obj;
            int8_t max = -128;
            obj.target = 0;

            for (int j = 0; j < num_class; j++)
            {
                if (max < dataset[i * num_element + OBJECT_T_INDEX + j])
                {
                    max = dataset[i * num_element + OBJECT_T_INDEX + j];
                    obj.target = j;
                }
            }

            float x = float(dataset[i * num_element + OBJECT_X_INDEX] - zero_point) * scale;
            float y = float(dataset[i * num_element + OBJECT_Y_INDEX] - zero_point) * scale;
            float w = float(dataset[i * num_element + OBJECT_W_INDEX] - zero_point) * scale;
            float h = float(dataset[i * num_element + OBJECT_H_INDEX] - zero_point) * scale;

            if (rescale)
            {
                obj.x = CLIP(int(x * width), 0, width);
                obj.y = CLIP(int(y * height), 0, height);
                obj.w = CLIP(int(w * width), 0, width);
                obj.h = CLIP(int(h * height), 0, height);
            }
            else
            {
                obj.x = CLIP(int(x), 0, width);
                obj.y = CLIP(int(y), 0, height);
                obj.w = CLIP(int(w), 0, width);
                obj.h = CLIP(int(h), 0, height);
            }
            obj.confidence = confidence;
            if (num_obj[obj.target] >= top_n)
            {
                yolo_obj_list[obj.target].sort(_object_comparator_reverse);
                if (obj.confidence > yolo_obj_list[obj.target].front().confidence)
                {
                    yolo_obj_list[obj.target].pop_front();
                    yolo_obj_list[obj.target].emplace_front(obj);
                }
            }
            else
            {
                yolo_obj_list[obj.target].emplace_front(obj);
                num_obj[obj.target]++;
            }
        }
    }

    std::forward_list<yolo_t> result;

    for (int i = 0; i < num_class; i++)
    {
        if (!yolo_obj_list[i].empty())
        {
            _soft_nms_obeject_detection(yolo_obj_list[i], nms);
            result.splice_after(result.before_begin(), yolo_obj_list[i]);
        }
    }

    result.sort(_object_comparator); // left to right

    return result;
}
