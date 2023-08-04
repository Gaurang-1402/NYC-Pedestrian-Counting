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
using std::max;
using std::min;

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "../ttn-esp32/include/TheThingsNetwork.h"
#include "../../../NYC_pedestrian_counter/main/config.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_system.h"

#include <vector>

#include "dsp_platform.h"
#include "esp_dsp.h"
#include "ekf.h"

#include <vector>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "esp_timer.h"

// Assume tracking_utils.h and kalman_box_tracker.h are defined with necessary functions

#include "ekf_helper.hpp"

static const char *TAG = "yolo";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gEvent = true;
static bool gReturnFB = true;
static bool debug_mode = false;

#define CONFIDENCE 50
#define IOU 45

const uint16_t box_color[] = {0x0000, 0xFFFF, 0x07E0, 0x001F, 0xF800, 0xF81F, 0xFFE0, 0x07FF, 0x07FF, 0x07FF, 0x07FF};

uint8_t mac[6];
char chipId[13];
/*
we use the esp_efuse_mac_get_default() function which gives us the base MAC address
(in the form of a 6-byte array) that is unique to each ESP32 chip.
This can be used as a unique identifier for your ESP32.
*/
void get_chip_id(char *chipId)
{
    uint8_t baseMac[6];
    esp_efuse_mac_get_default(baseMac);
    sprintf(chipId, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

std::vector<std::vector<double>> getMeasurements(std::forward_list<yolo_t> yolo_list)
{
    std::vector<std::vector<double>> measurements;
    for (const auto &yolo : yolo_list)
    {
        std::vector<double> measurement;
        measurement.reserve(5);
        measurement.push_back(static_cast<double>(yolo.x));
        measurement.push_back(static_cast<double>(yolo.y));
        measurement.push_back(static_cast<double>(yolo.w));
        measurement.push_back(static_cast<double>(yolo.h));
        measurements.push_back(measurement);
    }
    return measurements;
}

/* ====================================================================== */

// Start by defining points and lines to represent the counting lines

struct Point
{
    int x, y;
    Point(int _x, int _y) : x(_x), y(_y) {}
    Point() : x(0), y(0) {} // Default constructor
    // Other members...
};

struct Line
{
    Point p1, p2;
    Line(Point _p1, Point _p2) : p1(_p1), p2(_p2) {}
};

// Global pedestrian count for each line
int pedCountHorizontal = 0;
int pedCountVertical = 0;
int pedCountDiagonal = 0;

Line horizontalLine(Point(0, 0), Point(0, 0));
Line verticalLine(Point(0, 0), Point(0, 0));
Line diagonalLine(Point(0, 0), Point(0, 0));

// Function to find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(const Point &p, const Point &q, const Point &r)
{
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (val == 0)
        return 0; // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool onSegment(const Point &p, const Point &q, const Point &r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;

    return false;
}

// Function that returns true if line segment 'p1q1' and 'p2q2' intersect.
bool doIntersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2)
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
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

std::unordered_map<int, Point> lastPositions;
std::unordered_map<int, bool> crossedLine;

// Implement the C++ version of the Sort class
class Sort
{
public:
    Sort(int max_age = 2, int min_hits = 2) : max_age(max_age), min_hits(min_hits), frame_count(0) {}

    std::vector<std::vector<double>> update(const std::vector<std::vector<double>> &dets, float s_time)
    {
        frame_count++;
        std::vector<std::vector<double>> trks;
        // std::vector<std::vector<double>> trks(trackers.size(), std::vector<double>(5, 0));
        std::vector<int> to_del;
        std::vector<std::vector<double>> ret;

        for (size_t t = 0; t < trackers.size(); ++t)
        {
            float dt = (esp_timer_get_time() - s_time) / 10000000.0f;
            std::vector<double> pos = trackers[t].predict(dt);
            trks.push_back({pos[0], pos[1], pos[2], pos[3], 0});

            if (std::isnan(pos[0]) || std::isnan(pos[1]) || std::isnan(pos[2]) || std::isnan(pos[3]))
            {
                to_del.push_back(t);
            }
        }

        std::vector<std::vector<double>> temp_trks;
        temp_trks.reserve(trks.size() - to_del.size());
        for (size_t i = 0; i < trks.size(); ++i)
        {
            if (std::find(to_del.begin(), to_del.end(), static_cast<int>(i)) == to_del.end())
            {
                temp_trks.emplace_back(std::move(trks[i]));
            }
        }

        trks = std::move(temp_trks);

        trackers.erase(
            std::remove_if(trackers.begin(), trackers.end(), [&](const KalmanBoxTracker &tracker)
                           {
                               size_t index = &tracker - &trackers[0];
                               bool shouldDelete = std::find(to_del.begin(), to_del.end(), static_cast<int>(index)) != to_del.end();

                               // remove object from lastPositions if it's being deleted from trackers
                               if (shouldDelete)
                               {
                                   lastPositions.erase(tracker.id);
                               }

                               return shouldDelete; }),
            trackers.end()

        );

        auto [matched, unmatched] =
            associate_detections_to_trackers(dets, trks, 0.0);

        auto &[unmatched_dets, unmatched_trks] = unmatched;

        for (size_t t = 0; t < trackers.size(); ++t)
        {
            if (std::find(unmatched_trks.begin(), unmatched_trks.end(), static_cast<int>(t)) == unmatched_trks.end())
            {
                auto x = [t](const std::vector<int> &pair)
                { return pair[1] == static_cast<int>(t); };

                int d = matched[static_cast<size_t>(std::find_if(matched.begin(), matched.end(), x) - matched.begin())][0];

                if (!dets[d].empty())
                {

                    std::vector<double> state = trackers[t].get_state();
                    int id = trackers[t].id;
                    printf("ID: %d\n", id);
                    int centerX = dets[d][0] + (dets[d][2] / 2);
                    int centerY = dets[d][1] + (dets[d][3] / 2);

                    // Check for intersection with each line and increment count
                    printf("CrossedLine[%d]: %d\n", id, crossedLine[id]);
                    if (lastPositions.count(id) > 0 && !crossedLine[id])
                    {
                        Point lastPos = lastPositions[id];
                        if (doIntersect(lastPos, Point(centerX, centerY), horizontalLine.p1, horizontalLine.p2))
                        {
                            pedCountHorizontal++;
                            crossedLine[id] = true;
                        }
                        if (doIntersect(lastPos, Point(centerX, centerY), verticalLine.p1, verticalLine.p2))
                        {
                            pedCountVertical++;
                            crossedLine[id] = true;
                        }
                        if (doIntersect(lastPos, Point(centerX, centerY), diagonalLine.p1, diagonalLine.p2))
                        {
                            pedCountDiagonal++;
                            crossedLine[id] = true;
                        }
                    }

                    // Update last position of this object
                    lastPositions[id] = Point(centerX, centerY);
                    float dt = (esp_timer_get_time() - s_time) / 10000000.0f;
                    trackers[t].update(dets[d], dt);
                }
            }
        }

        for (auto i : unmatched_dets)
        {
            KalmanBoxTracker tracker(dets[i]);
            trackers.emplace_back(tracker);
        }

        const size_t tracker_size = trackers.size();

        std::vector<KalmanBoxTracker> temp_trackers2;

        for (size_t i = 0; i < tracker_size; ++i)
        {
            std::vector<double> d = trackers[i].get_state();
            if ((trackers[i].time_since_update < 5) && (trackers[i].hit_streak >= min_hits || frame_count <= min_hits))
            {
                ret.emplace_back(std::vector<double>(d.begin(), d.end()));
                ret.back().push_back(trackers[i].id + 1);
            }
            if (trackers[i].time_since_update <= max_age)
            {
                temp_trackers2.emplace_back(trackers[i]);
            }
            else // if a tracker is not being moved to temp_trackers2, it is essentially being removed, so remove it from lastPositions as well
            {
                lastPositions.erase(trackers[i].id);
            }
        }
        trackers = std::move(temp_trackers2);

        return ret;
    }

    std::vector<KalmanBoxTracker>
        trackers;

private:
    int max_age;
    int min_hits;
    int frame_count;
};

/* LoRa setup */
// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'idf.py menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.
// Copy the below hex strings from the TTN console (Applications > Your application > End devices
// > Your device > Activation information)
// AppEUI (sometimes called JoinEUI)
const char *appEui = APPEUI;
// DevEUI
const char *devEui = DEVEUI;
// AppKey
const char *appKey = APPKEY;
static TheThingsNetwork ttn;
const unsigned TX_INTERVAL = 20;
static uint8_t msgData[] = "Hello, world";
void sendMessage(void *pvParameter)
{
    printf("Sending message...\n");
    /* Prepare the payload */
    /*
    In this code, we're sending a payload that is 7 bytes long: the first 6 bytes are the MAC address and the last byte is the count.
    This is a very compact representation that fits well within the constraints of LoRaWAN.
    On the receiving side (TTN), we receive a 7-byte array.
    We need to parse this back into a MAC address and count.
    Note: This example assumes that that count is in the range 0-255.
    If it can be larger, we may need to use more bytes to represent it, and adjust the code accordingly.
    */
    uint8_t payload[7];
    for (int i = 0; i < 6; i++)
    {
        payload[i] = chipId[i];
    }
    printf("Pedestrian Count Max: %d\n", max(max(pedCountHorizontal, pedCountVertical), pedCountDiagonal));
    payload[6] = max(max(pedCountHorizontal, pedCountVertical), pedCountDiagonal);
    /* Send the payload */
    TTNResponseCode res = ttn.transmitMessage(payload, sizeof(payload));
    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
    if (res == kTTNSuccessfulTransmission)
    {
        pedCountHorizontal = 0;
        pedCountVertical = 0;
        pedCountDiagonal = 0;
        // clear measurement hashmap
        crossedLine.clear();
    }
    // vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
}
void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}
uint32_t ticks_now(void)
{
    TickType_t currentTick = xTaskGetTickCount();
    return pdTICKS_TO_MS(currentTick);
}
void print_time(void)
{
    uint32_t currentTime = ticks_now();
    printf("Current time in milliseconds: %u\n", currentTime);
}

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
    horizontalLine = Line(Point(0, h / 2), Point(w, h / 2)); // horizontal line
    verticalLine = Line(Point(w / 2, 0), Point(w / 2, h));   // vertical line
    diagonalLine = Line(Point(0, 0), Point(w, h));           // diagonal line

    // Initialize tracked objects

    // SORT class (counting)
    Sort sort;
    int init_time = (int)(esp_timer_get_time() / 1000);

    while (true)
    {
        if (gEvent)
        {
            if ((int)(esp_timer_get_time() / 1000) - init_time > 20000)
            {
                print_time();
                sendMessage(NULL);
                init_time = (int)(esp_timer_get_time() / 1000);
            }
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
                float s_time = esp_timer_get_time();

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

                // YOLO list
                _yolo_list = nms_get_obeject_topn(output->data.int8, records, CONFIDENCE, IOU, w, h, records, num_class, scale, zero_point);

                // fb_gfx_drawFastHLine(frame, horizontalLine.p1.x, horizontalLine.p1.y, w, 0xFF0000); // black
                // Draw vertical line
                // fb_gfx_drawFastVLine(frame, verticalLine.p1.x, verticalLine.p1.y, h, 0x00FF00); // yellow
                // Draw diagonal line
                // fb_gfx_drawLine(frame, diagonalLine.p1.x, diagonalLine.p1.y, diagonalLine.p2.x, diagonalLine.p2.y, 0x0000FF); // Blue

                std::vector<std::vector<double>>
                    measurements = getMeasurements(_yolo_list);

                // float dt = (esp_timer_get_time() - s_time) / 1000000.0; // Time in seconds

                // printf("dt: %f\r\n", dt);

                std::vector<std::vector<double>> combined;
                combined.insert(combined.end(), measurements.begin(), measurements.end());

                measurements = sort.update(measurements, s_time);
                combined.insert(combined.end(), measurements.begin(), measurements.end());

                for (auto &measurement : measurements)
                {
                    printf("Measurement: %f, %f, %f, %f\r\n", measurement[0], measurement[1], measurement[2], measurement[3]);

                    // draw
                    // fb_gfx_drawRect(frame, measurement[0], measurement[1], measurement[2], measurement[3], 0xFF0000); // Red
                }

                // printf("Number of tracks: %d\r\n", sort.trackers.size());
                // Update SORT

                // Print pedestrian counts
                std::cout << "\033[1;33mPedestrian Count for Horizontal Line: " << pedCountHorizontal << "\033[0m\n";
                std::cout << "\033[1;33mPedestrian Count for Vertical Line: " << pedCountVertical << "\033[0m\n";
                std::cout << "\033[1;33mPedestrian Count for Diagonal Line: " << pedCountDiagonal << "\033[0m\n"; // Yellow

                printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", (dsp_end_time - dsp_start_time), (end_time - start_time), 0);
                bool found = false;

                if (std::distance(_yolo_list.begin(), _yolo_list.end()) > 0)
                {
                    int index = 0;
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
                        // fb_gfx_drawRect2(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h / 2, yolo.w, yolo.h, box_color[index % (sizeof(box_color) / sizeof(box_color[0]))], 4);
                        // fb_gfx_printf(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h / 2 - 5, 0x1FE0, 0x0000, "%s", g_yolo_model_classes[yolo.target]);
                        printf("        {\"class\": \"%d\", \"x\": %d, \"y\": %d, \"w\": %d, \"h\": %d, \"confidence\": %d},\n", yolo.target, yolo.x, yolo.y, yolo.w, yolo.h, yolo.confidence);
                        index++;
                    }
                    printf("    ]\n");
                }

                printf("Number of tracks: %d\r\n", sort.trackers.size());
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

    // Communication
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);
    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    memset(&spi_bus_config, 0, sizeof(spi_bus_config));
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);
    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);
    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);
    // Register callback for received messages
    ttn.onMessage(messageReceived);
    //    ttn.setAdrEnabled(false);
    //    ttn.setDataRate(kTTNDataRate_US915_SF7);
    //    ttn.setMaxTxPower(14);
    // esp_wifi_deinit();
    // esp_wifi_get_mac(WIFI_IF_STA, mac);
    get_chip_id(chipId);

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

    printf("Starting\n");

    if (ttn.join())
    {
        printf("Joined.\n");
        printf("Chip ID is %s\n", chipId);
        xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
        if (xQueueEvent)
            xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
        return 0;
    }
    else
    {
        printf("Join failed. Goodbye\n");
        return 0;
    }

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
    return oa.x > ob.x;
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
            obj.w = (obj.x + obj.w) > width ? (width - obj.x) : obj.w;
            obj.h = (obj.y + obj.h) > height ? (height - obj.y) : obj.h;
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
