/**
 * @file credentials.template.h
 * @brief Template de credenciais e configurações sensíveis.
 * @details Copie este arquivo para `include/credentials.h` e preencha os valores.
 *          O arquivo `include/credentials.h` está listado no .gitignore para evitar o
 *          versionamento de informações privadas (SSID, senhas, IPs, etc.).
 */

#ifndef CREDENTIALS_H_
#define CREDENTIALS_H_

// ---- WiFi ------------------------------------------------------------------
// SSID e senha da rede WiFi a qual o ESP32 irá se conectar no modo Station.
// Substitua pelos dados reais depois de copiar o template.
#define WIFI_SSID        "QironRobotics"
#define WIFI_PASSWORD    "B30_n4_&$col@"

// ---- UDP -------------------------------------------------------------------
// Porta local onde o dispositivo irá escutar comandos UDP.
#define UDP_LISTEN_PORT        4210

// IP remoto autorizado a enviar pacotes. Use "0.0.0.0" para aceitar qualquer origem.
#define UDP_ALLOWED_REMOTE_IP  "0.0.0.0"

// Tamanho máximo (bytes) aceito para um pacote de controle UDP
#define UDP_MAX_PACKET_SIZE     64

// ---- Opções Extras ---------------------------------------------------------
// Timeout (ms) padrão para conexão WiFi (pode ser usado em futuras melhorias)
#define WIFI_CONNECT_TIMEOUT_MS 10000

#endif // CREDENTIALS_H_
