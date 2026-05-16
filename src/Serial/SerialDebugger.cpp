/**
 * @file SerialDebugger.cpp
 * @brief Debugger Serial  
 * @author Bruno Gabriel Flores Sampaio,
 * @date 6 de abril de 2024
 */

#include "SerialDebugger.h"

/* Semafaro para controle de fluxo de dados */
SemaphoreHandle_t serialDebuggerMutex;

void serial_debugger_init( void ) {
  /* Inicia o mutex para controle do DEBUG_SERIAL */
  serialDebuggerMutex = xSemaphoreCreateMutex();
  /* Inicializa a USB-CDC nativa (ESP32-C3 Super Mini). */
  USB_BUS.begin(USB_BUS_BAUDRATE);
  /* Non-blocking TX: don't stall when no host has opened the CDC port. */
  USB_BUS.setTxTimeoutMs(0);
  /* Wait up to 2s for the host to open the CDC port so early logs are not lost. */
  uint32_t t0 = millis();
  while (!USB_BUS && (millis() - t0) < 2000) {
    delay(10);
  }
  DEBUG_SERIAL("DEBUG UART", "USB CDC iniciada em %d bps.", USB_BUS_BAUDRATE);
}

String buffer2String(const uint8_t* buffer, size_t length) {
  String result;
  for (size_t i = 0; i < length; ++i) {
    if (buffer[i] < 0x10)
      result += "0";
    result += String(buffer[i], HEX);
  }
  return result;
}