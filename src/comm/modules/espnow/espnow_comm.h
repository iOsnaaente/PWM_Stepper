/**
 * @file    espnow_comm.h
 * @brief   CommBase transport over ESP-NOW (broadcast, fixed channel).
 * @details Drop-in replacement for WiFiComm. The ESP-NOW frame payload IS
 *          the protocol packet (ProtoComm::pack output) - no extra wrapping.
 *          ProtocolComm does not need any change.
 */

#pragma once

#include "interfaces/comm_base.h"
#include "espnow_comm_utils.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class ESPNowComm : public CommBase {
  private:
    espnow_comm_config_t *cfg;
    QueueHandle_t         rx_Queue;

    // Singleton ptr for the C-style ESP-NOW callbacks. Only one ESPNowComm
    // can exist at a time (acceptable since the ESP-NOW driver is global).
    static ESPNowComm *s_instance;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    static void on_recv(const esp_now_recv_info_t *info,
                        const uint8_t *data, int len);
#else
    static void on_recv(const uint8_t *src_mac,
                        const uint8_t *data, int len);
#endif
    static void on_send(const uint8_t *mac_addr,
                        esp_now_send_status_t status);

  public:
    ESPNowComm(const char *description = nullptr,
               espnow_comm_config_t *config = nullptr);
    ~ESPNowComm();

    CommRet_t init()   override;
    CommRet_t deinit() override;
    CommRet_t start()  override;
    CommRet_t stop()   override;

    CommRet_t send(const CommMessage_t &msg,
                   uint32_t timeout_ms = 0) override;
    CommRet_t send_to(const CommMessage_t &msg,
                      const CommAddr_t &addr,
                      uint32_t timeout_ms = 0) override;
    CommRet_t receive(CommMessage_t &out_msg,
                      uint32_t timeout_ms = 0) override;
    CommRet_t receive_from(CommMessage_t &out_msg,
                           CommAddr_t &addr,
                           uint32_t timeout = 0) override;
    size_t    available() const override;
};
