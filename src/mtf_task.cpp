#include "mtf_task.h"
#include "config.h"
#include "globals.h"
#include "msp2_parser.h"
#include "range_validation.h"
#include <HardwareSerial.h>

extern HardwareSerial mtfSerial;

void mtf_task(void *parameter) {
    (void)parameter;
    Msp2Parser parser;
    Msp2Parser::Packet packet;
    for (;;) {
        unsigned bytes = 0;
        while (mtfSerial.available() > 0 && bytes++ < 192) {
            if (!parser.feed(static_cast<uint8_t>(mtfSerial.read()), packet)) continue;
            uint32_t now = millis();

            if (packet.command == 0x1F01 && packet.len == 5) {
                int32_t d = Msp2Parser::readI32LE(&packet.payload[1]);
                uint8_t quality = packet.payload[0];
                bool valid = range_validation::packetRangeOk(d, quality);
                if (xSemaphoreTake(mtx_sensor, pdMS_TO_TICKS(2)) == pdTRUE) {
                    shared_mtf.distance_mm = d;
                    shared_mtf.range_quality = quality;
                    shared_mtf.range_valid = valid;
                    shared_mtf.last_range_ms = now;
                    ++shared_mtf.range_frames;
                    shared_mtf.crc_errors = parser.crcErrors();
                    xSemaphoreGive(mtx_sensor);
                }
            } else if (packet.command == 0x1F02 && packet.len == 9) {
                uint8_t quality = packet.payload[0];
                int32_t x = Msp2Parser::readI32LE(&packet.payload[1]);
                int32_t y = Msp2Parser::readI32LE(&packet.payload[5]);
                if (xSemaphoreTake(mtx_sensor, pdMS_TO_TICKS(2)) == pdTRUE) {
                    shared_mtf.flow_x_raw = x;
                    shared_mtf.flow_y_raw = y;
                    shared_mtf.flow_quality = quality;
                    shared_mtf.flow_valid = quality >= MIN_FLOW_QUALITY_DIAG;
                    shared_mtf.last_flow_ms = now;
                    ++shared_mtf.flow_frames;
                    shared_mtf.y_probe_counts.add(x, y, quality, MIN_FLOW_QUALITY_DIAG, 200);
                    shared_mtf.guided_cal_counts.add(x, y, quality, 70, 200);
                    // POS estimation gets all quality-approved frames, not just
                    // the most recent frame sampled by nav_task at 50 Hz.
                    // Saturated or low-quality frames cannot pollute sums.
                    if (quality >= 70 && x >= -200 && x <= 200 &&
                        y >= -200 && y <= 200) {
                        shared_mtf.flow_sum_x += x;
                        shared_mtf.flow_sum_y += y;
                        ++shared_mtf.flow_good_frames;
                    }
                    shared_mtf.crc_errors = parser.crcErrors();
                    xSemaphoreGive(mtx_sensor);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS) > 0 ?
                   pdMS_TO_TICKS(SENSOR_PERIOD_MS) : 1);
    }
}
