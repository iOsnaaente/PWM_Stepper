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
  /* Inicializa a UART conectada ao conversor USB-Serial da placa */
  USB_BUS.begin(USB_BUS_BAUDRATE, SERIAL_8N1, USB_RXD0_PIN, USB_TXD0_PIN);
  DEBUG_SERIAL("DEBUG UART", "UART debug iniciada em %d bps (RX=%d, TX=%d).", USB_BUS_BAUDRATE, (int)USB_RXD0_PIN, (int)USB_TXD0_PIN);
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