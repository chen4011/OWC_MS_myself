#ifndef REMOTEFETCHDATA_SETUPINIT_H_
#define REMOTEFETCHDATA_SETUPINIT_H_

#include <esp_now.h>

void initWiFi();
void initEspNow(esp_now_send_cb_t onDataSend, esp_now_recv_cb_t onDataRecv, uint8_t *broadcastAddress);

#endif