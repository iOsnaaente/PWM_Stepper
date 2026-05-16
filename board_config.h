/**
 * @file board_config.h
 * @brief Gerenciador das configurações relacionadas a placa 
 * do sistema de controle de acesso.
 * 
 * @details Neste arquivo, deve ser colocado as pinagens e 
 * configurações globais do projeto.
 *   
 * @date 2025.1
 * @author Bruno Gabriel Flores Sampaio,
 */


#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#include "Arduino.h"

#include "nvs_flash.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "time.h"
#include "math.h"

#include "functional"
#include "vector"
#include "map"

#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "credentials.h"


/** 
 * @brief Mensagem de debug para serial. 
 * @details É usado um Mutex para não haver conflitos de uso de memórias compartilhadas e deadlocks
 * Para desativar o debug serial, deve ser definido a Macrodefinição DEBUG_SERIAL_DESLIGADO
 * 
 * @example `#define DEBUG_SERIAL_COMPLETO`  -> Printa o nome do arquivo e linha + mensagem de debug
 * @example `#define DEBUG_SERIAL_REDUZIDO`  -> Printa somente a mensagem de debug
 * @example `#define DEBUG_SERIAL_DESLIGADO` -> Desliga o print de mensagens de debug 
 */

// #define DEBUG_SERIAL_COMPLETO 
#define DEBUG_SERIAL_REDUZIDO 
// #define DEBUG_SERIAL_DESLIGADO 


/**
 * @brief GPIO do LED da placa 
 */ 
#define USB_BUS             Serial
#define USB_BUS_NUM         UART_NUM_0 
#define USB_BUS_BAUDRATE    115200
#define USB_BUFF_SIZE       1024
// ESP32-C3 Super Mini: USB-UART bridge on GPIO20 (RX) and GPIO21 (TX)
#define USB_RXD0_PIN        ((gpio_num_t)(GPIO_NUM_20))
#define USB_TXD0_PIN        ((gpio_num_t)(GPIO_NUM_21))

// On ESP32-C3 Super Mini the on-board LED is on GPIO8
#define LED_BOARD           ((gpio_num_t)GPIO_NUM_8)

// --- Motores (ESP32-C3 Super Mini + A4988) ---
// Motor 1: STEP1 -> GPIO0, DIR1 -> GPIO4
#define M1_DIR_PIN        ((gpio_num_t)(GPIO_NUM_4))
#define M1_VEL_PIN        ((gpio_num_t)(GPIO_NUM_0))

// Motor 2: STEP2 -> GPIO1, DIR2 -> GPIO5
#define M2_DIR_PIN        ((gpio_num_t)(GPIO_NUM_5))
#define M2_VEL_PIN        ((gpio_num_t)(GPIO_NUM_1))

// Enable compartilhado para ambos drivers de stepper
#define ENABLE_PIN        ((gpio_num_t)(GPIO_NUM_10))

// ---------------------------------------------------------------------------
// OLED 0.91" I2C Display (SSD1306)
// ESP32-C3 Super Mini: SDA = GPIO8, SCL = GPIO9
#define OLED_SCL_PIN      ((gpio_num_t)(GPIO_NUM_9))
#define OLED_SDA_PIN      ((gpio_num_t)(GPIO_NUM_8))




#endif /* BOARD_CONFIG_H_ */