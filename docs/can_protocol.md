# Protocolo CAN — Blue Mechanic V1

## Visão Geral

A comunicação CAN utiliza o periférico TWAI do ESP32-S3 (modo on-chip) em configuração de slave. O master é um Teensy 4.1 rodando Zephyr RTOS. A topologia é ponto-a-ponto com 1 master e até 10 slaves, cada slave com um ID de node único (1–127).

O protocolo usa frames CAN padrão de 11 bits (standard identifiers) com DLC máximo de 8 bytes.

---

## Configuração de Hardware

| Parâmetro     | Valor Padrão | Pino ESP32-S3 |
|---------------|-------------|---------------|
| CAN TX        | GPIO 48     | GPIO_NUM_48   |
| CAN RX        | GPIO 47     | GPIO_NUM_47   |
| Bitrate       | 500 kbps    | —             |
| Node ID       | 1           | —             |

### IDs CAN

Cada slave possui três faixas de IDs base configuráveis:

| Base            | Padrão  | Finalidade                        |
|-----------------|---------|-----------------------------------|
| `can_command_base_id` | `0x200` | Comandos do master para o slave   |
| `can_status_base_id`  | `0x280` | Status do slave para o master     |
| `can_event_base_id`   | `0x300` | Eventos do slave para o master    |

O ID efetivo de cada slave é calculado somando o `node_id` à base:

```
own_command_id  = can_command_base_id + node_id
own_status_id   = can_status_base_id  + node_id
own_event_id    = can_event_base_id   + node_id
```

**Exemplo para node_id = 1:**
- Comando: `0x201`
- Status: `0x281`
- Evento: `0x301`

### Filtro de Hardware

O filtro TWAI do ESP32-S3 é configurado para aceitar **apenas** o ID exclusivo do próprio slave (`own_command_id`), com máscara `0x7FF` (match exato de 11 bits). Isso elimina a necessidade de filtrar frames de outros slaves no software.

O broadcast (`can_command_base_id` puro, sem node_id) **não** é aceito pelo filtro de hardware. O master deve enviar comandos direcionados para cada slave individualmente.

---

## Formato do Frame CAN

Todos os frames usam o formato padrão de 11 bits:

| Campo       | Bits          | Valor                          |
|-------------|---------------|--------------------------------|
| Standard ID | 11 bits       | ID efetivo do slave (base + node_id) |
| DLC         | 4 bits        | 1–8 bytes de dados             |
| Data        | 0–64 bits     | Payload do frame               |
| RTR         | 1 bit         | Data frame (0)                 |
| IDE         | 1 bit         | Standard frame (0)             |

---

## Direção da Comunicação

| Direção     | ID de Origem     | ID de Destino      | Descrição                              |
|-------------|------------------|--------------------|----------------------------------------|
| Master → Slave | `own_command_id` | —                  | Comandos do master para o slave        |
| Slave → Master | —                | `own_status_id`    | Respostas de status e eventos          |
| Slave → Master | —                | `own_event_id`     | Eventos assíncronos (heartbeat, ACK)   |

---

## Sequenciamento de Comandos

Cada comando CAN pode incluir um byte opcional de sequência no final do payload. O byte de sequência é o **último byte** do frame, além do payload esperado pelo opcode.

### Como funciona

- Se o DLC do frame for **maior** que o tamanho esperado do opcode, o byte extra é o número de sequência.
- Se o DLC for **igual** ao tamanho esperado, não há sequência (comportamento backward-compatible).
- O slave rastreia `last_cmd_seq` por opcode. Comandos com sequência ≤ `last_cmd_seq` são descartados como duplicatas.
- Sequência `0` é tratada como sem sequenciamento (sempre processado).

### Exemplo

Para `CAN_OP_SPEED` (tamanho esperado = 2 bytes: opcode + level):
- Frame com DLC=2: `[0x11, 3]` → processado (sem sequência)
- Frame com DLC=3: `[0x11, 3, 5]` → processado se `5 > last_cmd_seq`, senão ignorado

O master deve incrementar o número de sequência para cada novo comando enviado a um slave.

---

## OpCodes de Comando (Master → Slave)

### `0x01` — `CAN_OP_PING`

**Tamanho mínimo:** 3 bytes

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Opcode (`0x01`)   |
| 1    | Arg0 (dado arbitrário) |
| 2    | Arg1 (dado arbitrário) |
| 3 (opcional) | Sequência |

**Resposta:** `CAN_EVT_PONG` com os mesmos Arg0 e Arg1.

---

### `0x02` — `CAN_OP_STATUS_REQUEST`

**Tamanho mínimo:** 1 byte

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Opcode (`0x02`)   |

**Resposta:** `CAN_EVT_STATUS` com payload de 8 bytes (ver seção Status).

---

### `0x10` — `CAN_OP_ENABLE`

**Tamanho mínimo:** 2 bytes

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Opcode (`0x10`)                       |
| 1    | Enable flag (`0x01` = ligar, `0x00` = desligar) |
| 2 (opcional) | Sequência                        |

**Ação:** Liga/desliga os drivers de passo via `hardware_set_driver_enable()`.

**Resposta:** `CAN_EVT_ACK` ou `CAN_EVT_ERROR`.

---

### `0x11` — `CAN_OP_SPEED`

**Tamanho mínimo:** 2 bytes

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Opcode (`0x11`)                       |
| 1    | Nível de velocade (1–5)               |
| 2 (opcional) | Sequência                        |

**Níveis de velocidade:**

| Level | `move_delay_us` | Velocidade relativa |
|-------|-----------------|---------------------|
| 1     | 2000 µs         | Mais lenta          |
| 2     | 800 µs          | Lenta               |
| 3     | 400 µs          | Média               |
| 4     | 150 µs          | Rápida              |
| 5     | 50 µs           | Mais rápida         |

**Resposta:** `CAN_EVT_ACK` ou `CAN_EVT_ERROR`.

---

### `0x20` — `CAN_OP_MOVE`

**Tamanho mínimo:** 6 bytes

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Opcode (`0x20`)                       |
| 1    | Eixo (`'X'`, `'Y'` ou `'Z'`)          |
| 2    | Steps byte 0 (LSB)                    |
| 3    | Steps byte 1                        |
| 4    | Steps byte 2                        |
| 5    | Steps byte 3 (MSB)                    |
| 6 (opcional) | Sequência                        |

**Ação:** Enfileira comando de movimento relativo em `motion_queue`. O slave executa o movimento de forma assíncrona.

**Resposta:** `CAN_EVT_ACK` (enfileirado) ou `CAN_EVT_ERROR`.

---

### `0x21` — `CAN_OP_HOME`

**Tamanho mínimo:** 2 bytes

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Opcode (`0x21`)                       |
| 1    | Eixo (`'X'`, `'x'`, `'Y'`, `'y'`, `'Z'`, `'z'`) |
| 2 (opcional) | Sequência                        |

**Ação:** Enfileira comando de homing no eixo especificado. O movimento Z pode levar até ~24 segundos (30000 passos com yield a cada 4096).

**Resposta:** `CAN_EVT_ACK` (enfileirado) ou `CAN_EVT_ERROR`.

---

### `0x30` — `CAN_OP_LASER`

**Tamanho mínimo:** 3 bytes

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Opcode (`0x30`)                       |
| 1    | Índice do laser (1 ou 2)              |
| 2    | Nível PWM (0–255)                     |
| 3 (opcional) | Sequência                        |

**Ação:** Configura o nível PWM do laser selecionado via `hardware_set_laser_level()`.

**Resposta:** `CAN_EVT_ACK` ou `CAN_EVT_ERROR`.

---

### `0x31` — `CAN_OP_FAN`

**Tamanho mínimo:** 2 bytes

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Opcode (`0x31`)                       |
| 1    | Modo do fan                           |
| 2 (opcional) | Sequência                        |

**Modos do fan:**

| Valor | Modo                  | Ação                          |
|-------|-----------------------|-------------------------------|
| 0     | `FAN_MODE_MANUAL_OFF` | Fan desligado                 |
| 1     | `FAN_MODE_MANUAL_ON`  | Fan ligado                    |
| 2     | `FAN_MODE_AUTO`       | Fan controlado por temperatura|

**Resposta:** `CAN_EVT_ACK` ou `CAN_EVT_ERROR`.

---

## Eventos e Respostas (Slave → Master)

### `0x80` — `CAN_EVT_HEARTBEAT`

Enviado periodicamente pelo slave a cada 1000 ms quando o barramento CAN está online.

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Evento (`0x80`)   |
| 1    | Node ID do slave  |
| 2–7  | Reservado (0)     |

---

### `0x81` — `CAN_EVT_PONG`

Resposta ao `CAN_OP_PING`.

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Evento (`0x81`)   |
| 1    | Arg0 do ping      |
| 2    | Arg1 do ping      |
| 3–7  | Reservado (0)     |

---

### `0x82` — `CAN_EVT_STATUS`

Resposta ao `CAN_OP_STATUS_REQUEST`. Payload de 8 bytes.

| Byte | Conteúdo                              |
|------|---------------------------------------|
| 0    | Evento (`0x82`)                       |
| 1    | Node ID do slave                      |
| 2    | Flags de estado (bitmask)             |
| 3    | Nível do laser 1 (0–255)              |
| 4    | Nível do laser 2 (0–255)              |
| 5    | Flags do fan (bitmask)                |
| 6    | Temperatura atual (int8_t, °C × 100)  |
| 7    | Nível de velocade (1–5)               |

### Byte 2 — Flags de estado (bitmask):

| Bit | Máscara  | Significado                    |
|-----|----------|--------------------------------|
| 0   | `0x01`   | `drivers_enabled`              |
| 1   | `0x02`   | `z_bloqueado`                  |
| 2   | `0x04`   | `alarme_z_ativo`               |
| 3   | `0x08`   | `temp_valid`                   |
| 4   | `0x10`   | `tmc_uart_ready`               |
| 5   | `0x20`   | `can_online`                   |

### Byte 5 — Flags do fan (bitmask):

| Bit | Máscara  | Significado                    |
|-----|----------|--------------------------------|
| 0   | `0x01`   | `fan_output_on`                |
| 1–7 | —        | `fan_mode` (shiftado 1 bit à esquerda) |

---

### `0x83` — `CAN_EVT_ACK`

Confirmação de que um comando foi processado com sucesso.

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Evento (`0x83`)   |
| 1    | Opcode do comando |
| 2    | 0 (sucesso)       |
| 3–7  | Reservado (0)     |

---

### `0x84` — `CAN_EVT_DONE`

Indica que um comando de movimento foi concluído (o eixo chegou ao destino).

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Evento (`0x84`)   |
| 1    | Opcode do comando |
| 2    | 0 (sucesso)       |
| 3–7  | Reservado (0)     |

---

### `0xE0` — `CAN_EVT_ERROR`

Indica que um comando falhou.

| Byte | Conteúdo          |
|------|-------------------|
| 0    | Evento (`0xE0`)   |
| 1    | Opcode do comando |
| 2    | Código do erro (`esp_err_t`) |
| 3–7  | Reservado (0)     |

**Códigos de erro comuns:**

| Valor              | Significado                    |
|--------------------|--------------------------------|
| `ESP_OK` (0)       | Sucesso (usado em ACK/DONE)    |
| `ESP_ERR_INVALID_ARG` | Argumento inválido           |
| `ESP_ERR_INVALID_STATE` | Slave CAN offline          |
| `ESP_ERR_NOT_SUPPORTED` | Opcode desconhecido        |
| `ESP_ERR_NO_MEM`    | Falha de memória (queue cheia)|

---

## Validação de Configuração CAN

O slave valida as configurações CAN antes de iniciar o periférico TWAI:

| Parâmetro     | Restrição                          |
|---------------|------------------------------------|
| `node_id`     | 1–127                              |
| `can_bitrate` | 125000, 250000, 500000, 1000000   |
| `can_command_base_id + 127` | ≤ `0x7FF` (limite de 11 bits) |
| `can_status_base_id + 127`  | ≤ `0x7FF`                       |
| `can_event_base_id + 127`   | ≤ `0x7FF`                       |

Se a configuração for inválida, o slave não inicia o TWAI e reporta `ESP_ERR_INVALID_ARG`.

---

## Implementação do Master (Teensy 4.1 + Zephyr)

### Inicialização

1. Configurar o controlador CAN do Teensy 4.1 (MCAN) com o mesmo bitrate dos slaves (padrão: 500 kbps).
2. Configurar o filtro de recepção para aceitar todos os IDs de comando dos slaves (faixa `[can_command_base_id, can_command_base_id + 127]`).
3. Configurar os IDs de transmissão para status e eventos de cada slave.

### Envio de Comandos

Para enviar um comando a um slave:

```c
// Exemplo: enviar CAN_OP_MOVE para node_id = 1
uint16_t target_id = can_command_base_id + node_id;  // 0x201
uint8_t payload[8] = {0x20, 'X', 0x00, 0x10, 0x00, 0x00, seq, 0};
// payload[0] = opcode CAN_OP_MOVE (0x20)
// payload[1] = eixo 'X'
// payload[2-5] = steps (little-endian, int32_t)
// payload[6] = sequência (opcional)

can_send(target_id, payload, 7);  // DLC=7 (com sequência)
// ou
can_send(target_id, payload, 6);  // DLC=6 (sem sequência)
```

### Recebimento de Respostas

O master deve configurar filtros separados para:
- IDs de status (`can_status_base_id + node_id`)
- IDs de evento (`can_event_base_id + node_id`)

Ou usar um único filtro que aceite a faixa de eventos/status de todos os slaves.

### Tratamento de Respostas

| Evento recebido | Ação esperada pelo master                              |
|-----------------|-------------------------------------------------------|
| `CAN_EVT_ACK`   | Comando enfileirado com sucesso                       |
| `CAN_EVT_DONE`  | Movimento concluído                                   |
| `CAN_EVT_ERROR` | Comando falhou — verificar byte 2 para o código de erro |
| `CAN_EVT_PONG`  | Resposta ao ping — verificar Arg0 e Arg1              |
| `CAN_EVT_STATUS`| Atualizar estado do slave                             |
| `CAN_EVT_HEARTBEAT` | Manter conexão ativa                               |

### Timeout e Retransmissão

- Se nenhum evento for recebido em **500 ms** após o envio de um comando, considerar o comando como falho (timeout).
- O master pode retransmitir o comando com a mesma sequência — o slave irá ignorá-lo como duplicata.
- Para reenviar após timeout, usar um **novo número de sequência**.

### Exemplo de Máquina de Estados do Master

```
ESTADO_IDLE → envia comando → ESPERA_RESPOSTA
    │                                    │
    │                              recebe ACK/DONE/ERROR
    │                                    │
    └────────────────────────────────────┘
        │
        ├── ACK/DONE → volta a IDLE
        ├── ERROR  → reporta erro, volta a IDLE
        └── timeout → retransmite (nova seq) ou reporta falha
```

---

## Considerações de Timing

| Operação                  | Tempo Máximo     | Observação                              |
|---------------------------|------------------|-----------------------------------------|
| CAN_OP_PING → PONG        | < 5 ms           | Resposta síncrona                       |
| CAN_OP_STATUS_REQUEST     | < 5 ms           | Resposta síncrona                       |
| CAN_OP_ENABLE             | < 5 ms           | Resposta síncrona                       |
| CAN_OP_SPEED              | < 5 ms           | Resposta síncrona                       |
| CAN_OP_MOVE               | < 5 ms           | Resposta síncrona (enfileirado)         |
| CAN_OP_HOME (X/Y)         | < 5 ms           | Resposta síncrona (enfileirado)         |
| CAN_OP_HOME (Z)           | < 5 ms           | Resposta síncrona (enfileirado), execução assíncrona até ~24s |
| CAN_OP_LASER              | < 5 ms           | Resposta síncrona                       |
| CAN_OP_FAN                | < 5 ms           | Resposta síncrona                       |
| Heartbeat                 | A cada 1000 ms   | Assíncrono                              |

---

## Resumo do Protocolo

| Aspecto              | Detalhe                                                        |
|----------------------|----------------------------------------------------------------|
| Modo                 | Standard 11-bit identifiers                                    |
| Bitrate              | 500 kbps (configurável: 125k, 250k, 500k, 1000k)             |
| Topologia            | 1 master + até 10 slaves                                       |
| Filtragem            | Hardware: match exato por node; Software: verificação de ID   |
| Broadcast            | Não suportado (comandos direcionados)                          |
| Sequenciamento       | Opcional, byte extra no final do payload                       |
| Confirmação          | ACK para comandos rápidos; DONE para movimentos                |
| Heartbeat            | 1000 ms                                                        |
| Payload máximo       | 8 bytes                                                        |
| Endianness           | Little-endian para multi-bytes                                 |