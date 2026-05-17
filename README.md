# PWM_Stepper
Controle de motores de passo usando bloco de comando PWM dentro do ESP32.

## 📡 Conectividade (WiFi + UDP)
O firmware conecta-se a uma rede WiFi (modo Station) e escuta comandos de controle via UDP.

### Arquivo de Credenciais
Para evitar expor SSID/Senha no repositório, foi adicionado um mecanismo de template:

1. Copie o arquivo `include/credentials.template.h` para `include/credentials.h`:
	 ```
	 cp include/credentials.template.h include/credentials.h
	 ```
2. Edite `include/credentials.h` e preencha:
	 - `WIFI_SSID`
	 - `WIFI_PASSWORD`
3. Compile e faça o upload normalmente com o PlatformIO.

> O arquivo `include/credentials.h` está listado no `.gitignore` e não será versionado.

### Macros Principais de Configuração
| Macro | Função |
|-------|--------|
| `WIFI_SSID` | Nome da rede WiFi para conexão |
| `WIFI_PASSWORD` | Senha da rede WiFi |
| `UDP_LISTEN_PORT` | Porta UDP onde o firmware escuta comandos |
| `UDP_MAX_PACKET_SIZE` | Tamanho máximo de pacote UDP aceito |

### Formatos de Pacote de Controle
O listener UDP aceita múltiplos formatos para dirigir as rodas independentemente:
1. Binário compacto: 2 bytes (L, R) normalizados em `int8_t` (–127..127) + opcional checksum XOR (3º byte)
2. ASCII CSV: `"<left>,<right>"` (ex: `"0.5,-0.25"`)
3. Legado 1 byte: `F`,`B`,`L`,`R` ou qualquer outro (stop)

### Segurança Básica
Para evitar acessos indevidos:
- Use uma VLAN ou rede isolada em ambientes críticos.
- Avalie adicionar autenticação no protocolo caso seja exposto fora de rede local.

## 🛠 Debug Serial
Use as macros `DEBUG_SERIAL` para mensagens. Modos disponíveis em `board_config.h`:
```
// #define DEBUG_SERIAL_COMPLETO
#define DEBUG_SERIAL_REDUZIDO
// #define DEBUG_SERIAL_DESLIGADO
```

## ⚙ Motores de Passo
Cada motor é controlado via PWM usando o periférico LEDC. A classe `Stepper` permite:
- Ajuste de velocidade normalizada (−1.0 a 1.0)
- Controle de torque (enable/disable)
- Conversão de velocidade para frequência de passos

## 🤖 Classe Robot
Abstrai dois motores (esquerdo/direito) e fornece:
- `drive(vel, turn)` mistura diferencial
- `drive_wheels(left, right)` controle direto
- `stop()`

## 📂 Estrutura do Projeto
```
include/
	credentials.template.h  <- Template de credenciais
	credentials.h           <- Sua cópia (não versionada)
src/
	main.cpp                <- Inicialização WiFi + loop de status + tarefa UDP
	Robot/                  <- Lógica de alto nível
	Stepper/                <- Driver simplificado de motor
	Serial/                 <- Infra de debug
board_config.h            <- Pinos e includes globais
platformio.ini            <- Configuração PlatformIO
```

## 🚀 Próximos Passos Sugeridos
- Implementar autenticação leve nos pacotes (ex: token de 1 byte + checksum)
- Adicionar OTA (já há comentários no `platformio.ini` para espota)
- Criar script Python para enviar comandos binários/CSV facilmente

## 📄 Licença
Adicione aqui a licença do projeto (ex: MIT, Apache 2.0, etc.).

