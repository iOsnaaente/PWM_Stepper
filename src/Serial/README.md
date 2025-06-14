# SerialController && SerialDebugger

## Autor

- **Company:** Qiron Robotics
- **Author:** Bruno Gabriel Flores Sampaio
- **Data:** 6 de abril de 2024

---

# SerialController - DEFASADO 

--- 

# SerialDebugger

Este projeto fornece uma biblioteca para depuração serial usando a UART0, permitindo o envio de mensagens de debug personalizadas. O código suporta diferentes níveis de detalhes de debug, conforme definido pelas macros `DEBUG_SERIAL_COMPLETO`, `DEBUG_SERIAL_REDUZIDO`, e `DEBUG_SERIAL_DESLIGADO`.

## Estrutura do Projeto

- **`SerialDebugger.h`**: Arquivo principal que contém as definições e protótipos para as funcionalidades de depuração serial. Inclui macros para diferentes níveis de debug e proteção de acesso à porta serial através de um mutex.

## Funcionalidades

1. **Depuração Serial Completa** (`DEBUG_SERIAL_COMPLETO`):
   - Imprime o tipo de mensagem, o nome do arquivo e a linha de código de onde a mensagem foi chamada.
   - Protege o acesso à porta serial usando um mutex.
   - Exemplo de uso:
     ```cpp
     DEBUG_SERIAL("INFO", "Inicialização concluída");
     ```

2. **Depuração Serial Reduzida** (`DEBUG_SERIAL_REDUZIDO`):
   - Imprime apenas o tipo e a mensagem de debug.
   - Também utiliza o mutex para garantir que apenas uma mensagem seja enviada por vez.
   - Exemplo de uso:
     ```cpp
     DEBUG_SERIAL("ERRO", "Falha na comunicação");
     ```

3. **Depuração Serial Desligada** (`DEBUG_SERIAL_DESLIGADO`):
   - Desativa todas as mensagens de debug, permitindo que o código seja compilado sem saída de debug.

## Funções

### `serialDebugger_init()`

Inicializa as interfaces UART para comunicação com dispositivos externos e para depuração.

- **Descrição**: Configura as taxas de transmissão (baud rate) e outros parâmetros necessários para a comunicação serial. Garante que as interfaces estejam prontas para transmissão e recepção de dados.
- **Uso**: Deve ser chamada no início do programa para garantir a configuração adequada das interfaces seriais.

```cpp
void serialDebugger_init();
```

### `buffer2String(const uint8_t* buffer, size_t length)`

Converte um buffer de bytes em uma representação hexadecimal em formato de string.

- **Descrição**: Útil para depurar buffers, especialmente quando é necessário inspecionar conteúdo em formato hexadecimal.
- **Exemplo de uso**:
  ```cpp
  Serial.print("O buffer contém: " + buffer2String(buffer, len));
  ```
- **Parâmetros**:
  - `buffer`: Ponteiro para o buffer de bytes.
  - `length`: Número de bytes no buffer.
- **Retorno**: Um objeto `String` com a representação hexadecimal dos bytes.

```cpp
String buffer2String(const uint8_t* buffer, size_t length);
```

## Configuração

Para habilitar ou desabilitar diferentes níveis de debug, defina uma das macros a seguir antes de incluir o arquivo `SerialDebugger.h`:

- `DEBUG_SERIAL_COMPLETO`: Ativa mensagens de debug completas.
- `DEBUG_SERIAL_REDUZIDO`: Ativa mensagens de debug reduzidas.
- `DEBUG_SERIAL_DESLIGADO`: Desativa todas as mensagens de debug.

Exemplo:
```cpp
#define DEBUG_SERIAL_COMPLETO
#include "SerialDebugger.h"
```


