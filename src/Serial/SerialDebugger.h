/**
 * @file SerialDebugger.h
 * @brief Header do Debugger Serial  
 * @details Este arquivo contém as definição e prototipos das implementações
 *        - de debug serial utilizando a UART0. Para utilizar o debug é
 *        - necessário se chamar a Macro DEBUG_SERIAL( TIPO, MESSAGE )  
 * @author Bruno Gabriel Flores Sampaio,
 * @date 6 de abril de 2024
 */

#ifndef SERIAL_DEBUGGER_H_
#define SERIAL_DEBUGGER_H_

#include "../board_config.h"
#include "Display/debug_display.h"

#include "driver/uart.h"

/* Mutex para proteger o acesso à porta serial */
extern SemaphoreHandle_t serialDebuggerMutex;

#ifdef DEBUG_SERIAL_COMPLETO
  #define DEBUG_SERIAL( TIPO, FORMAT, ... )                                                                                  \
    if (xSemaphoreTake(serialDebuggerMutex, ( TickType_t ) pdMS_TO_TICKS(5) ) == pdTRUE) {                                    \
      char msg_buff[255];                                                                                                     \
      snprintf( msg_buff, sizeof(msg_buff), FORMAT, ##__VA_ARGS__ );                                                          \
      String log_message = "[" + String( TIPO) + "] " + String(__FILE__) + " [" + String(__LINE__) + "]: " + MESSAGE + "\n";  \
      uart_write_bytes(USB_UART_NUM, log_message.c_str(), strlen(log_message.c_str()));                                       \
      xSemaphoreGive(serialDebuggerMutex);                                                                                    \
    }
#endif

#ifdef DEBUG_SERIAL_REDUZIDO
  #define DEBUG_SERIAL( TIPO, FORMAT, ... )                                             \
    if (xSemaphoreTake(serialDebuggerMutex, (TickType_t) pdMS_TO_TICKS(5)) == pdTRUE) { \
      char msg_buff[256];                                                               \
      snprintf( msg_buff, sizeof(msg_buff), FORMAT, ##__VA_ARGS__ );                    \
      String log_message = "[" + String(TIPO) + "]: " + String(msg_buff) + "\r\n";      \
      uart_write_bytes(UART_NUM_0, log_message.c_str(), strlen(log_message.c_str()));   \
      debug_display_push(TIPO, msg_buff);                                               \
      xSemaphoreGive(serialDebuggerMutex);                                              \
    }
#endif 

#ifdef DEBUG_SERIAL_DESLIGADO
  #define DEBUG_SERIAL( TIPO, MESSAGE ) {};
#endif 

/**
 * @brief Inicializa as interfaces UART para comunicação.
 * @details Esta função configura as interfaces UART usadas para comunicação com dispositivos 
 * externos e para fins de depuração. Ela estabelece as taxas de transmissão (baud rates)
 * e outros parâmetros necessários para as portas seriais USB_BUS.
 * Além disso, garante que as interfaces seriais estejam prontas para transmissão e recepção
 * de dados.
 * @note Esta função deve ser chamada no início do programa para garantir que as interfaces
 *       seriais estejam devidamente inicializadas antes de qualquer comunicação ocorrer.
 */
void serial_debugger_init( void );

/**
 * @brief Converte um buffer de bytes em uma representação de string hexadecimal.
 * @details Função itera sobre cada byte no buffer, converte-o para o seu equivalente
 * em string hexadecimal, e, anexa ao resultado da string. Para bytes menores que 0x10,
 * adiciona um zero à esquerda para manter um formato consistente de dois caracteres
 * para cada byte.
 * @note Essa função é util para ser utilizada em debugging de buffers: 
 *  ```Serial.print( "O buffer contem: " + buffer2String(buffer, len) )```
 * @param buffer Ponteiro para o buffer de bytes a ser convertido.
 * @param length O número de bytes no buffer a ser convertido.
 * @return Um objeto String contendo a representação hexadecimal dos bytes no buffer.
 */
String buffer2String( const uint8_t* buffer, size_t length);


#endif /* SERIAL_DEBUGGER_H_ */