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

uint8_t mac[6];
char boardId[18]; // MAC address will be 17 characters long

const uint16_t box_color[] = {0x1FE0, 0x07E0, 0x001F, 0xF800, 0xF81F, 0xFFE0};
// Global counter
int current_id = 0;

// TODO come up with a clever way for unique ID
// Function to generate a new ID
int get_new_id()
{
    return current_id++;
}

// Start by defining a C structure to represent a centroid and a tracked object
typedef struct Centroid
{
    int x;
    int y;
} Centroid;

typedef struct TrackedObject
{
    int id;
    Centroid centroid;
    Centroid last_centroid; // Add this field to store the last position of the object
    int disappeared;
} TrackedObject;

std::vector<TrackedObject> objects;

/* ====================================================================== */

// Start by defining points and lines to represent the counting lines

struct Point
{
    int x, y;
    Point(int _x, int _y) : x(_x), y(_y) {}
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

// Function to find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (val == 0)
        return 0; // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
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

bool crosses_line(Centroid last_centroid, Centroid centroid, Line line)
{
    // Check if the line segment from last_centroid to centroid crosses the line.
    // Return true if it does, false otherwise.

    Point p1(last_centroid.x, last_centroid.y);
    Point q1(centroid.x, centroid.y);

    return doIntersect(p1, q1, line.p1, line.p2);
}

/* ====================================================================== */

/* ====================================================================== */

void register_object(Centroid centroid)
{
    TrackedObject new_object;
    new_object.id = get_new_id();
    new_object.centroid = centroid;
    new_object.last_centroid = centroid;
    new_object.disappeared = 0;
    objects.push_back(new_object);
}

void deregister_object(int index)
{
    objects.erase(objects.begin() + index);
}
// Maximum distance between an object's old centroid and a new centroid for them
// to be considered the same object.
const int max_distance = 50;

void update(std::vector<Centroid> new_centroids, Line horizontalLine, Line verticalLine, Line diagonalLine)
{
    if (new_centroids.empty())
    {
        for (auto &object : objects)
        {
            object.disappeared++;

            if (object.disappeared > 10)
            {
                deregister_object(object.id);
            }
        }
        return;
    }

    if (objects.empty())
    {
        for (const auto &centroid : new_centroids)
        {
            register_object(centroid);
        }
        std::cout << "\033[1;32mRegistered " << new_centroids.size() << " new objects.\033[0m\n";
        return;
    }

    std::vector<std::vector<int>> distances(objects.size(), std::vector<int>(new_centroids.size()));
    for (int i = 0; i < objects.size(); ++i)
    {
        for (int j = 0; j < new_centroids.size(); ++j)
        {
            distances[i][j] = std::abs(objects[i].centroid.x - new_centroids[j].x) +
                              std::abs(objects[i].centroid.y - new_centroids[j].y);
        }
    }

    for (int i = 0; i < objects.size(); ++i)
    {
        int min_distance = max_distance + 1;
        int closest_j = -1;
        for (int j = 0; j < new_centroids.size(); ++j)
        {
            if (distances[i][j] < min_distance)
            {
                min_distance = distances[i][j];
                closest_j = j;
            }
        }

        if (closest_j != -1)
        {
            // Before updating the current centroid, store its value in last_centroid
            objects[i].last_centroid = objects[i].centroid;

            // Now you can update the current centroid
            objects[i].centroid = new_centroids[closest_j];
            objects[i].disappeared = 0;

            // Check if the object crossed the lines.
            if (crosses_line(objects[i].last_centroid, objects[i].centroid, horizontalLine))
            {
                pedCountHorizontal++;
            }
            if (crosses_line(objects[i].last_centroid, objects[i].centroid, verticalLine))
            {
                pedCountVertical++;
            }
            if (crosses_line(objects[i].last_centroid, objects[i].centroid, diagonalLine))
            {
                pedCountDiagonal++;
            }

            new_centroids[closest_j].x = new_centroids[closest_j].y = -1;
            std::cout << "\033[1;34mObject " << objects[i].id << " updated with new centroid.\033[0m\n";
        }
        else
        {
            objects[i].disappeared++;
            std::cout << "\033[1;31mObject " << objects[i].id << " has disappeared.\033[0m\n";
        }
    }

    for (const auto &centroid : new_centroids)
    {
        if (centroid.x != -1 && centroid.y != -1)
        {
            register_object(centroid);
            std::cout << "\033[1;33mRegistered new object with centroid.\033[0m\n";
        }
    }
}

/* ====================================================================== */

std::forward_list<yolo_t> nms_get_obeject_topn(int8_t *dataset, uint16_t top_n, uint8_t threshold, uint8_t nms, uint16_t width, uint16_t height, int num_record, int8_t num_class, float scale, int zero_point);

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
    memcpy(payload, mac, 6);
    printf("Pedestrian Count Max: %d\n", max(max(pedCountHorizontal, pedCountVertical), pedCountDiagonal));
    payload[6] = max(max(pedCountHorizontal, pedCountVertical), pedCountDiagonal);

    /* Send the payload */
    TTNResponseCode res = ttn.transmitMessage(payload, sizeof(payload));

    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
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
    Line verticalLine = Line(Point(w / 2, 0), Point(w / 2, h));   // vertical line
    Line diagonalLine = Line(Point(0, 0), Point(w, h));           // diagonal line
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
                // int16_t num_element = num_class + OBJECT_T_INDEX;

                _yolo_list = nms_get_obeject_topn(output->data.int8, records, CONFIDENCE, IOU, w, h, records, num_class, scale, zero_point);

                fb_gfx_drawFastHLine(frame, horizontalLine.p1.x, horizontalLine.p1.y, w, 0xFF0000); // Red
                // Draw vertical line
                fb_gfx_drawFastVLine(frame, verticalLine.p1.x, verticalLine.p1.y, h, 0x00FF00); // Green
                // Draw diagonal line
                fb_gfx_drawLine(frame, diagonalLine.p1.x, diagonalLine.p1.y, diagonalLine.p2.x, diagonalLine.p2.y, 0x0000FF); // Blue

                std::vector<Centroid> centroids;
                for (const auto &object : _yolo_list)
                {
                    Centroid centroid;
                    centroid.x = object.x + object.w / 2;
                    centroid.y = object.y + object.h / 2;
                    centroids.push_back(centroid);
                }

                // Print pedestrian counts
                std::cout << "\033[1;33mPedestrian Count for Horizontal Line: " << pedCountHorizontal << "\033[0m\n";
                std::cout << "\033[1;33mPedestrian Count for Vertical Line: " << pedCountVertical << "\033[0m\n";
                std::cout << "\033[1;33mPedestrian Count for Diagonal Line: " << pedCountDiagonal << "\033[0m\n"; // Yellow

                update(centroids, horizontalLine, verticalLine, diagonalLine);

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
                        // fb_gfx_drawRect(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h / 2, yolo.w, yolo.h, 0x1FE0);
                        // fb_gfx_printf(frame, yolo.x - yolo.w / 2, yolo.y - yolo.h / 2 - 5, 0x1FE0, 0x0000, "%s", g_yolo_model_classes[yolo.target]);
                        printf("        {\"class\": \"%s\", \"x\": %d, \"y\": %d, \"w\": %d, \"h\": %d, \"confidence\": %d},\n", g_yolo_model_classes[yolo.target], yolo.x, yolo.y, yolo.w, yolo.h, yolo.confidence);
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
                // continue;
                // printf("Sending frame0\n");
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
                // printf("Sent frame0\n");
            }
            else if (gReturnFB)
            {
                // printf("Returning frame1\n");
                esp_camera_fb_return(frame);
            }
            else
            {
                // printf("Freeing frame2\n");
                free(frame);
                // printf("Freed frame2\n");
            }

            if (xQueueResult)
            {
                // printf("Sending result3\n");
                xQueueSend(xQueueResult, NULL, portMAX_DELAY);
            }
        }
        else
        {
            printf("no event\n");
        }
        // print_time();
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("we should never get here\n");
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
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

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
    // static tflite::MicroMutableOpResolver<14> micro_op_resolver;
    // micro_op_resolver.AddConv2D();
    // micro_op_resolver.AddReshape();
    // micro_op_resolver.AddPad();
    // micro_op_resolver.AddAdd();
    // micro_op_resolver.AddSub();
    // micro_op_resolver.AddRelu();
    // micro_op_resolver.AddMaxPool2D();
    // micro_op_resolver.AddConcatenation();
    // micro_op_resolver.AddQuantize();
    // micro_op_resolver.AddTranspose();
    // micro_op_resolver.AddLogistic();
    // micro_op_resolver.AddMul();
    // micro_op_resolver.AddStridedSlice();
    // micro_op_resolver.AddResizeNearestNeighbor();

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

    if (ttn.join())
    {
        printf("Joined.\n");
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

    // xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
    // if (xQueueEvent)
    //     xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);

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

            int x = int(float(float(dataset[i * num_element + OBJECT_X_INDEX] - zero_point) * scale) * width);
            int y = int(float(float(dataset[i * num_element + OBJECT_Y_INDEX] - zero_point) * scale) * height);
            int w = int(float(float(dataset[i * num_element + OBJECT_W_INDEX] - zero_point) * scale) * width);
            int h = int(float(float(dataset[i * num_element + OBJECT_H_INDEX] - zero_point) * scale) * height);

            obj.x = CLIP(x, 0, width);
            obj.y = CLIP(y, 0, height);
            obj.w = CLIP(w, 0, width);
            obj.h = CLIP(h, 0, height);
            obj.confidence = int(float(confidence) * 100);
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