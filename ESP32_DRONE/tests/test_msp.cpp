#include "msp2_parser.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
int main() {
    const uint8_t flow[] = {0x24,0x58,0x3C,0x00,0x02,0x1F,0x09,0x00,
                            0x38,0,0,0,0,0,0,0,0,0x3D};
    const uint8_t dist[] = {0x24,0x58,0x3C,0x00,0x01,0x1F,0x05,0x00,
                            0x01,0xff,0xff,0xff,0xff,0xAE};
    Msp2Parser p;
    Msp2Parser::Packet out;
    int ok = 0;
    for (uint8_t b : flow) if (p.feed(b, out)) {
        ++ok;
        assert(out.command == 0x1F02);
        assert(out.len == 9);
        assert(out.payload[0] == 0x38);
        assert(Msp2Parser::readI32LE(&out.payload[1]) == 0);
    }
    for (uint8_t b : dist) if (p.feed(b, out)) {
        ++ok;
        assert(out.command == 0x1F01);
        assert(out.len == 5);
        assert(Msp2Parser::readI32LE(&out.payload[1]) == -1);
    }
    assert(ok == 2);
    uint8_t bad[sizeof(flow)];
    for (size_t i = 0; i < sizeof(flow); ++i) bad[i] = flow[i];
    bad[8] ^= 0x01;
    for (uint8_t b : bad) assert(!p.feed(b, out));
    assert(p.crcErrors() == 1);
    // Recover after corrupted packet.
    int recover = 0;
    for (uint8_t b : flow) if (p.feed(b, out)) ++recover;
    assert(recover == 1);
    printf("PASS: MSPv2 flow/range, signed int32, CRC rejection, recovery\n");
}
