# Blue Mechanic V1 — App Desktop PyQt6

Aplicação desktop profissional em Python e PyQt6 para controle, parametrização, telemetria em tempo real e diagnósticos do hardware **Blue Mechanic V1** (ESP32-S3 com drivers de passo TMC2209, lasers PWM, sensor térmico DS18B20, ventoinha e encoders magnéticos AS5600).

---

## 🧭 Cinemática dos Eixos

- **Eixo C (Base Rotativa)**: Base rotativa principal indexada como 0 (encoder I2C Bus 0, RMT canal 0).
- **Eixo A (Pivot acoplado na base rotativa)**: Pivot montado sobre a base C, indexado como 1 (encoder I2C Bus 1, RMT canal 1). Contém **2 lasers (Laser 1 e Laser 2)** posicionados em orientação colinear oposta (180° no mesmo eixo óptico/pivot), girando junto com o pivot.
- **Eixo Z (Atuador Linear)**: Atuador linear de elevação e curso vertical indexado como 2.

---

## 🚀 Funcionalidades

- **Comunicação Dupla**:
  - **Porta Serial (COM / UART)**: Transmissão de comandos ASCII (`STATUS`, `MOVE`, `MOVE_F`, `HOME`, `SETHOME`, `SPEED`, `ACCEL`, `LASER`, `FAN`, `DRIVER`, etc.) com parsing e telemetria contínua.
  - **PeakCAN (PCAN-Basic)**: Suporte completo ao protocolo CAN 11-bit padrão com decodificação de `HEARTBEAT` (0x80), `STATUS` (0x82), `PONG` (0x81), `ACK` (0x83), `DONE` (0x84) e `ERROR` (0xE0).
  - **Simulador Virtual Integrado**: Permite testar movimentação, lasers, aquecimento térmico e sniffer sem precisar do hardware conectado fisicamente.
- **Dashboard em Tempo Real**:
  - Ângulos C e A (°), posição Z (passos e mm) com barra de progresso visual.
  - Temperatura com sensor DS18B20 e estados de alerta.
  - Status dos drivers de passo, alarme do switch Z e ventoinha.
- **Movimentação & Jogging**:
  - Jog direcional cross-pad para C/A e botões de subida/descida para Z com tamanhos de passo configuráveis (0.5°, 1°, 5°, 10°, 45° ou passos Z).
  - Comandos diretos de Homing (`HOME C`, `HOME A`, `HOME Z`, `HOME TODOS`).
  - Gravação de zero na NVS (`SETHOME C`, `SETHOME A`, `SETLENGTH Z`).
- **Periféricos & Gestão Térmica**:
  - Sliders de alta precisão com porcentagem e valor 8-bit (0-255) para Laser 1 (Frontal) e Laser 2 (Oposto 180°) com presets rápidos (0%, 25%, 50%, 75%, 100%).
  - Controle de modos da ventoinha (Desligado, Ligado, Automático por temperatura).
- **Gerenciador de Parâmetros (NVS & TMC2209 & CAN)**:
  - Passos por volta, limites de velocidade e aceleração para Eixo C (Base), Eixo A (Pivot) e Eixo Z (Linear).
  - Configuração UART dos drivers TMC2209 (Corrente `irun`, `ihold`, `ihold_delay`, `SpreadCycle`, `Microsteps`).
  - Configurações da rede CAN (`node_id`, `can_bitrate`, `base_ids`).
- **Terminal Serial & Sniffer CAN**:
  - Console com histórico de comandos enviados e recebidos com formatação colorida.
  - Tabela de sniffer de frames CAN com decodificação de opcodes em tempo real.

---

## 📦 Instalação e Execução

### 1. Instalar Dependências
```bash
pip install -r python_app/requirements.txt
```

*(Dependências principais: `PyQt6`, `pyserial`, `python-can`)*

### 2. Executar o Aplicativo
```bash
python python_app/app.py
```
ou
```bash
python -m python_app.app
```

---

## 🔌 Modos de Conexão

1. **Porta Serial (COM)**:
   - Selecione `Porta Serial (COM)` no topo da tela.
   - Escolha a porta COM correspondente ao ESP32-S3 e o baud rate (padrão `115200`).
   - Clique em **Conectar**.

2. **PeakCAN (PCAN-Basic)**:
   - Certifique-se de ter o driver PCAN instalado no Windows (driver PCAN-USB da PEAK-System).
   - Selecione `PeakCAN (PCAN-Basic)` e o canal desejado (ex: `PCAN_USBBUS1`) e bitrate (padrão `500000`).
   - Clique em **Conectar**.

3. **Simulador Virtual**:
   - Selecione `Simulador Virtual` e clique em **Conectar** para testar toda a interface de forma autônoma.
