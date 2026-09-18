# Blue Mechanic V1 — App Desktop PyQt6

Aplicação desktop profissional em Python e PyQt6 para controle, parametrização, telemetria em tempo real e diagnósticos do hardware **Blue Mechanic V1** (ESP32-S3 com drivers de passo TMC2209, lasers PWM, sensor térmico DS18B20, ventoinha e encoders magnéticos AS5600).

---

## 🧭 Cinemática dos Eixos

- **Eixo C (Base Rotativa)**: Base rotativa principal indexada como 0 (encoder I2C Bus 0, RMT canal 0).
- **Eixo A (Pivot acoplado na base rotativa)**: Pivot montado sobre a base C, indexado como 1 (encoder I2C Bus 1, RMT canal 1). Contém **2 lasers (Laser 1 e Laser 2)** posicionados em orientação colinear oposta (180° no mesmo eixo óptico/pivot), girando junto com o pivot.
- **Eixo Z (Atuador Linear)**: Atuador linear de elevação e curso vertical indexado como 2.

---

## 🚀 Funcionalidades

- **Quatro backends de comunicação**:
  - **ESP32-S3 Serial Direta (COM / UART)**: Transmissão de comandos ASCII (`STATUS`, `MOVE`, `MOVE_F`, `HOME`, `SETHOME`, `SPEED`, `ACCEL`, `LASER`, `FAN`, `DRIVER`, etc.) com parsing e telemetria contínua.
  - **Teensy USB/CAN (COM / CDC-ACM)**: Conexão serial própria com o bridge Teensy 4.1 e comandos compactos `M`, `MF`, `MS`, `MSF`, `H`, `E`, `S`, `L`, `F`, `P` e `R`. O app seleciona um node ESP32-S3 entre 1 e 10 e decodifica `STATUS`, `POS`, `HEARTBEAT`, `PONG`, `ACK`, `DONE`, `ERROR` e `TEENSY_ERROR`.
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
  - Sliders de alta precisão com porcentagem e valor 12-bit (0-4095) para Laser 1 (Frontal) e Laser 2 (Oposto 180°) com presets rápidos (0%, 25%, 50%, 75%, 100%).
  - Controle de modos da ventoinha (Desligado, Ligado, Automático por temperatura).
- **Gerenciador de Parâmetros (NVS & TMC2209 & CAN)**:
  - Passos por volta, limites de velocidade e aceleração para Eixo C (Base), Eixo A (Pivot) e Eixo Z (Linear).
  - Configuração UART dos drivers TMC2209 (Corrente `irun`, `ihold`, `ihold_delay`, `SpreadCycle`, `Microsteps`).
  - Configurações da rede CAN (`node_id`, `can_bitrate`, `base_ids`).
- **Terminal Serial & Sniffer CAN**:
  - Console com histórico de comandos enviados e recebidos com formatação colorida.
  - Tabela de sniffer de frames CAN com decodificação de opcodes em tempo real.
- **Modo Broadcast Teensy (Node 0 / CAN ID 0x200)**:
  - Checkbox `📢 Broadcast (Node 0)` no cabeçalho de conexão para disparar todos os comandos de movimentação e periféricos simultaneamente para todos os nós ESP32-S3 do barramento.
  - Mantém o nó de telemetria selecionado para leituras de status sem gerar colisões no barramento CAN.
- **Automação de Testes e Sequenciador em Loop (`🔁 Automação & Testes`)**:
  - Editor de scripts ASCII com suporte a delays temporizados (`WAIT 500`), pausas e comentários.
  - Execução em **Loop Contínuo (Infinito)**, repetição de N vezes ou passo a passo.
  - Presets prontos com 1 clique (Varredura Sincronizada C/A/Z, Estresse de Lasers, Ciclo de Homing, Broadcast em Todos os Nós, Escalonamento de Velocidades).
  - Painel de métricas em tempo real (Loops Concluídos, Passos, Comandos TX, Falhas e Tempo Decorrido) e log detalhado.

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

### 3. Compilar para Executável Standalone (.exe)
Para gerar o executável autônomo sem dependências externas (tudo empacotado em um único `.exe`):
```bash
pyinstaller --noconfirm --clean blue_mechanic.spec
```
O executável final estará disponível em:
`dist/BlueMechanic.exe` (ou `dist/app.exe`).

---

## 🔌 Modos de Conexão

1. **ESP32-S3 Serial Direta**:
   - Selecione `ESP32-S3 Serial Direta` no topo da tela.
   - Escolha a porta COM correspondente ao ESP32-S3 e o baud rate (padrão `115200`).
   - Clique em **Conectar**.

2. **Teensy USB/CAN**:
   - Selecione `Teensy USB/CAN`.
   - Escolha a porta COM USB do Teensy, o baud rate `115200` e o node ESP32-S3 de destino (`1` a `10`).
   - Marque `📢 Broadcast (Node 0)` se desejar transmitir todos os comandos de atuação para todos os nós simultaneamente via CAN ID `0x200`. O app continuará solicitando `R <node>` periodicamente ao nó monitorado.
   - O terminal muda automaticamente para exemplos do protocolo Teensy. O movimento sincronizado usa `MS/MSF` e o opcode CAN `0x23`; gravação de NVS e perfil individual por eixo continuam reportados como não suportados.
   - Utilize a aba **`🔁 Automação & Testes`** para executar rotinas e varreduras em loop contínuo.

3. **PeakCAN (PCAN-Basic)**:
   - Certifique-se de ter o driver PCAN instalado no Windows (driver PCAN-USB da PEAK-System).
   - Selecione `PeakCAN (PCAN-Basic)` e o canal desejado (ex: `PCAN_USBBUS1`) e bitrate (padrão `500000`).
   - Clique em **Conectar**.

4. **Simulador Virtual**:
   - Selecione `Simulador Virtual` e clique em **Conectar** para testar toda a interface de forma autônoma.
