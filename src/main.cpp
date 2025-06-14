/**
  * @file PWM_Controller.cpp
  * @author Bruno Gabriel Flores Sampaio
  * @author Icaro Marques de Campos 
  * @date 13 de Junho de 2025
 **/

#include "../board_config.h"

#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"

#include "BluetoothSerial.h"

Stepper *motor_esquerdo;
Stepper *motor_direito;
Robot   *robot;

void bluetooth_listener_Task(void *pvParameters);

BluetoothSerial SerialBT;

void setup() {
    // Inicializa o Debugger Serial 
    serial_debugger_init( );
    DEBUG_SERIAL( "SERIAL INIT", "Serial de  debug inicializado." );
    DEBUG_SERIAL( "SERIAL INIT", "Baudrate: %d", USB_BUS_BAUDRATE );
    
    motor_esquerdo = new Stepper( M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0 );
    motor_direito = new Stepper( M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_1, LEDC_TIMER_0 );
    robot = new Robot( *motor_esquerdo, *motor_direito );
    // Parar 
    robot->stop( );

    // Inicializa Bluetooth
    SerialBT.begin("ESP32-Robot");
    xTaskCreatePinnedToCore(bluetooth_listener_Task, "BluetoothControl", 8192, NULL, 1, NULL, 0);
    DEBUG_SERIAL("BLUETOOTH", "Bluetooth iniciado. Nome: ESP32-Robot");

}


void bluetooth_listener_Task(void *pvParameters) {
    while (true) {
        while (SerialBT.available()) {
            uint8_t c = SerialBT.read();
            DEBUG_SERIAL( "BT_CMD", "Recebido: %d", c );
            if ( c == 70 ) {        
                robot->drive( 1.0, 0.0 );
            }
            else if ( c == 66 ) {        
                robot->drive( -1.0, 0.0 );
            }
            else if ( c == 76 ) {        
                robot->drive( 0.0, -1.0 );
            }
            else if ( c == 82 ) {        
                robot->drive( 0.0, 1.0 );
            } else {
                robot->drive( 0.0, 0.0 );
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void loop() {
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t free_heap  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t largest    = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    DEBUG_SERIAL("RAM", "Heap total: %u KB", total_heap / 1024);
    DEBUG_SERIAL("RAM", "Heap livre: %u KB", free_heap / 1024);
    DEBUG_SERIAL("RAM", "Maior bloco livre: %u KB", largest / 1024);
    vTaskDelete(NULL);
}