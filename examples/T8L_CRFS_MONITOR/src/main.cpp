// RadioMaster T8L -> ELRS receiver (CRSF UART) -> ESP32
// Bench monitor ONLY. KHONG XUAT PWM TO ESC. THAO TAT CA CANH QUAT.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <AlfredoCRSF.h>

static constexpr int RX_PIN = 16;  // ESP32 RX <- RX module TX
static constexpr int TX_PIN = 17;  // ESP32 TX -> RX module RX (optional telemetry)
static constexpr uint32_t PRINT_PERIOD_MS = 200;

HardwareSerial radioSerial(2);
AlfredoCRSF crsf;

static int prevCh[11] = {};
static bool hadLink = false;
static uint32_t lastPrint = 0;
static uint32_t lastValidMs = 0;

void setup() {
    // Khong khoi tao ESP32Servo, va khong phat ESC signals.
    for (int pin : {12, 13, 14, 27}) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    Serial.begin(115200);
    delay(700);
    radioSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
    crsf.begin(radioSerial);
    Serial.println("T8L ELRS / CRSF CHANNEL MONITOR (NO MOTOR PWM)");
    Serial.println("Test: Roll/Pitch/Throttle/Yaw, SA/SB/SC/SD one at a time.");
    Serial.println("Never infer CH5/CH6 without observing real output.");
}

void loop() {
    crsf.update();
    bool link = crsf.isLinkUp();
    int ch[11] = {};
    if (link) {
        bool sane = true;
        for (int i=1; i<=10; ++i) {
            ch[i] = crsf.getChannel(i);
            if (i <= 4 && (ch[i] < 900 || ch[i] > 2100)) sane = false;
        }
        link = sane;
        if (link) lastValidMs = millis();
    }
    if (link != hadLink) {
        Serial.printf("LINK %s\n", link ? "UP" : "LOST");
        hadLink = link;
        if (!link) { for (int i=1;i<=10;++i) prevCh[i] = 0; }
    }
    if (millis() - lastPrint >= PRINT_PERIOD_MS) {
        lastPrint = millis();
        Serial.printf("t=%lu LINK=%u age=%lu | ", (unsigned long)millis(),
                      link ? 1u : 0u, (unsigned long)(millis()-lastValidMs));
        for (int i=1; i<=10; ++i) {
            if (link) Serial.printf("CH%d=%4d%s ", i, ch[i],
                 prevCh[i] && abs(ch[i] - prevCh[i]) >= 50 ? "*" : " ");
            else Serial.printf("CH%d=---- ", i);
        }
        Serial.println();
        if (link) { for (int i=1;i<=10;++i) prevCh[i] = ch[i]; }
    }
    delay(1);
}
