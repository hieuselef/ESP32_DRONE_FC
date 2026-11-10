#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// MSP v2: "$X<" + flags(1) + cmd(2 LE) + len(2 LE)
//         + payload(len) + CRC8 DVB-S2 (poly 0xD5)
class Msp2Parser {
public:
    static constexpr size_t MAX_PAYLOAD = 24;
    static constexpr size_t MAX_FRAME = 9 + MAX_PAYLOAD;

    struct Packet {
        uint16_t command = 0;
        uint8_t len = 0;
        uint8_t payload[MAX_PAYLOAD] = {};
    };

    bool feed(uint8_t b, Packet &out) {
        if (idx_ == 0 && b != '$') return false;
        if (idx_ == 1 && b != 'X') { restart(b); return false; }
        if (idx_ == 2 && b != '<') { restart(b); return false; }
        if (idx_ >= MAX_FRAME) { restart(b); return false; }
        buf_[idx_++] = b;
        if (idx_ < 8) return false;

        const uint16_t len = static_cast<uint16_t>(buf_[6]) |
                             (static_cast<uint16_t>(buf_[7]) << 8);
        if (len > MAX_PAYLOAD) { restart(b); return false; }
        const size_t frame_len = 9 + len;
        if (idx_ < frame_len) return false;

        uint8_t crc = 0;
        for (size_t i = 3; i + 1 < frame_len; ++i) {
            crc = crc8(crc, buf_[i]);
        }
        bool valid = crc == buf_[frame_len - 1];
        if (valid) {
            out.command = static_cast<uint16_t>(buf_[4]) |
                          (static_cast<uint16_t>(buf_[5]) << 8);
            out.len = static_cast<uint8_t>(len);
            memcpy(out.payload, &buf_[8], len);
        } else {
            ++crc_errors_;
        }
        idx_ = 0;
        return valid;
    }

    uint32_t crcErrors() const { return crc_errors_; }

    static int32_t readI32LE(const uint8_t *p) {
        uint32_t u = static_cast<uint32_t>(p[0]) |
                    (static_cast<uint32_t>(p[1]) << 8) |
                    (static_cast<uint32_t>(p[2]) << 16) |
                    (static_cast<uint32_t>(p[3]) << 24);
        int32_t s;
        memcpy(&s, &u, sizeof(s));
        return s;
    }

private:
    static uint8_t crc8(uint8_t crc, uint8_t data) {
        crc ^= data;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
        return crc;
    }
    void restart(uint8_t b) {
        idx_ = 0;
        if (b == '$') buf_[idx_++] = b;
    }
    uint8_t buf_[MAX_FRAME] = {};
    size_t idx_ = 0;
    uint32_t crc_errors_ = 0;
};
