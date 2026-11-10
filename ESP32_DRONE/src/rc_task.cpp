#include "rc_task.h"
#include "config.h"
#include "globals.h"
#include <AlfredoCRSF.h>

extern AlfredoCRSF crsf;

void rc_task(void *parameter) {
    (void)parameter;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(RC_PERIOD_MS) > 0 ? pdMS_TO_TICKS(RC_PERIOD_MS) : 1;
    for (;;) {
        crsf.update();
        bool ok = crsf.isLinkUp();  // AlfredoCRSF tu timeout khi mat kenh
        int channels[11] = {};
        if (ok) {
            for (int i = 1; i <= 10; i++) {
                channels[i] = crsf.getChannel(i);
                // Chi cac kenh su dung 1..6 la bat buoc.
                // Aux 7..10 co the khong duoc cau hinh trong tay dieu khien.
                if (i <= CH_MODE && (channels[i] < 950 || channels[i] > 2050)) ok = false;
            }
        }
        if (mtx_rc && xSemaphoreTake(mtx_rc, pdMS_TO_TICKS(2)) == pdTRUE) {
            shared_rc.link_ok = ok;
            if (ok) {
                for (int i = 1; i <= 10; ++i) shared_rc.ch[i] = channels[i];
                shared_rc.last_ok_ms = millis();
            }
            xSemaphoreGive(mtx_rc);
        }
        vTaskDelayUntil(&wake, period);
    }
}
