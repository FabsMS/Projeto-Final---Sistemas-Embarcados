# 🎯 Mesa Labirinto Controlada por Joystick

> Projeto Final da disciplina de **Sistemas Embarcados (2026.1)** — um sistema embarcado interativo em **ESP32** que inclina uma mesa de labirinto em dois eixos por meio de servomotores, comandados por um joystick analógico, com leitura de orientação por **MPU6050** e telemetria em tempo real para visualização.

<p align="center">
  <img src="https://img.shields.io/badge/Plataforma-ESP32--C6-E7352C?style=flat-square&logo=espressif&logoColor=white" alt="ESP32-C6">
  <img src="https://img.shields.io/badge/Framework-ESP--IDF-000000?style=flat-square&logo=espressif&logoColor=white" alt="ESP-IDF">
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-44883E?style=flat-square&logo=freebsd&logoColor=white" alt="FreeRTOS">
  <img src="https://img.shields.io/badge/Linguagem-C-555555?style=flat-square&logo=c&logoColor=white" alt="C">
  <img src="https://img.shields.io/badge/Sim-Wokwi-AA00FF?style=flat-square" alt="Wokwi">
  <img src="https://img.shields.io/badge/Status-Fase%202%20concluída-1f8b4c?style=flat-square" alt="Status">
</p>

---

## 📑 Sumário

- [Visão geral](#-visão-geral)
- [Demonstração](#-demonstração)
- [Hardware](#-hardware)
- [Mapa de pinos](#-mapa-de-pinos)
- [Arquitetura de software](#-arquitetura-de-software)
- [Detalhe dos módulos](#-detalhe-dos-módulos)
- [Formato de telemetria (UART)](#-formato-de-telemetria-uart)
- [Como compilar e gravar](#-como-compilar-e-gravar)
- [Roadmap das fases](#-roadmap-das-fases)
- [Estrutura do repositório](#-estrutura-do-repositório)
- [Equipe](#-equipe)

---

## 🔍 Visão geral

A mesa é sustentada por uma estrutura que permite movimento nos eixos **X** e **Y** através de dois servomotores 90G controlados pelo ESP32. O usuário inclina a mesa com um **joystick analógico** para guiar uma esfera de aço pelo labirinto até a saída.

Um sensor inercial **MPU6050** fixado na mesa mede continuamente os ângulos de **pitch** (X) e **roll** (Y), e esses dados são enviados ao computador via UART em formato JSON — prontos para alimentar um *gêmeo digital* da mesa em um painel de visualização.

O firmware é construído sobre **FreeRTOS**, com cada responsabilidade isolada em sua própria task e o estado do sistema compartilhado de forma segura por meio de um *mutex*.

**Principais características:**

- 🕹️ Leitura analógica do joystick com **média móvel de 16 amostras** para suavizar ruído
- ⚙️ Controle de servos via **LEDC PWM** (50 Hz, resolução de 13 bits) com **filtro exponencial** de movimento e *deadzone* no centro
- 📐 Cálculo de pitch/roll com **filtro complementar** (α = 0.96) combinando acelerômetro e giroscópio
- 🎯 **Calibração do giroscópio** na inicialização e *recalibração* via botão do joystick
- 💡 **LED de status** que sinaliza o fim do boot
- 📡 Telemetria periódica em **JSON pela UART** a 10 Hz
- 🧵 Arquitetura **multitarefa** com 4 tasks e estado protegido por *mutex*

---

## 🔧 Hardware

| Qtd. | Componente | Função |
|:---:|---|---|
| 1 | **ESP32-C6** (DevKitC-1) | Microcontrolador principal |
| 1 | **Joystick analógico** | Controle dos servos nos eixos X e Y |
| 2 | **Servo motor 90G** | Inclinação da mesa nos eixos X e Y |
| 1 | **MPU6050** | Sensor inercial de orientação (pitch e roll) |
| 1 | **LED** (+ resistor) | Indicação de status / fim do boot |

> 💡 O projeto foi desenvolvido e validado no simulador **Wokwi** com a placa **ESP32-C6** (núcleo único), o que influenciou as prioridades e os tamanhos de *stack* das tasks.

---

## 📌 Mapa de pinos

Pinagem conforme `main/diagram.json` e os `#define` de cada driver:

| Sinal | GPIO | Observação |
|---|:---:|---|
| Joystick — eixo horizontal (HORZ) | **GPIO 1** | `ADC1_CH1`, atenuação 12 dB |
| Joystick — eixo vertical (VERT) | **GPIO 4** | `ADC1_CH4`, atenuação 12 dB |
| Joystick — botão (SEL) | **GPIO 6** | Entrada com *pull-up* interno |
| Servo X — PWM | **GPIO 15** | `LEDC` canal 0 |
| Servo Y — PWM | **GPIO 2** | `LEDC` canal 1 |
| LED de status | **GPIO 10** | Em série com resistor |
| MPU6050 — SDA | **GPIO 8** | I²C `I2C_NUM_0` @ 400 kHz |
| MPU6050 — SCL | **GPIO 9** | I²C `I2C_NUM_0` @ 400 kHz |

> O endereço I²C do MPU6050 é `0x68` (AD0 → GND).

---

## 🧵 Arquitetura de software

Todas as tarefas compartilham uma única estrutura de estado (`system_data_t global_data`), protegida por um **mutex** (`data_mutex`). Isso evita condições de corrida quando uma task escreve e outra lê os mesmos dados (leitura do joystick, ângulos do MPU, limites de PWM etc.).

```
                       ┌────────────────────────────┐
                       │        app_main()          │
                       │  cria mutex + inicializa HW │
                       └─────────────┬──────────────┘
                                     │ cria tasks
        ┌──────────────┬─────────────┼──────────────┬──────────────┐
        ▼              ▼             ▼               ▼              
  ┌───────────┐  ┌───────────┐  ┌──────────┐  ┌────────────┐       
  │  JoyRead  │  │  Servos   │  │  Status  │  │  MPU6050   │       
  │  prio 3   │  │  prio 2   │  │  prio 1  │  │   prio 2   │       
  │  (2048)   │  │  (2048)   │  │  (2048)  │  │   (4096)   │       
  └─────┬─────┘  └─────▲─────┘  └────▲─────┘  └─────┬──────┘       
        │ escreve      │ lê          │ lê           │ escreve       
        │              │             │              │ ângulos       
        ▼              │             │              ▼ + JSON UART    
   ╔═══════════════════════════════════════════════════════╗
   ║          global_data  ──  protegido por  data_mutex   ║
   ╚═══════════════════════════════════════════════════════╝
```

| Task | Prioridade | Stack | Período | Responsabilidade |
|---|:---:|:---:|:---:|---|
| `joystick_read_task` | 3 | 2048 | 10 ms | Leitura ADC dos eixos + botão (média móvel + debounce) |
| `servo_control_task` | 2 | 2048 | 20 ms | Converte joystick → PWM e aplica filtro de suavização |
| `mpu6050_task` | 2 | 4096 | 10 ms | Lê IMU, calcula pitch/roll e envia JSON pela UART |
| `status_task` | 1 | 2048 | 500 ms | Pisca o LED no boot e imprime logs periódicos |

> A `mpu6050_task` recebe *stack* maior (4096) porque usa ponto flutuante e `printf`. A prioridade do joystick é a mais alta para garantir resposta de controle fluida mesmo em uma placa de núcleo único.

---

## 🧩 Detalhe dos módulos

### 🕹️ Joystick — `joystick.c`
- ADC em modo *oneshot* com atenuação de 12 dB (faixa de leitura até ~3,3 V).
- **Média móvel** de 16 amostras por eixo, com pré-carga de 50 leituras na inicialização para começar estável.
- Botão lido com **debounce por tempo**, capaz de detectar o gatilho usado para recalibração do sensor.

### ⚙️ Servos — `servo.c`
- PWM por **LEDC** a 50 Hz com resolução de 13 bits (período padrão de 20 ms para servos).
- **Zona morta** (*deadzone*) no centro do joystick para eliminar tremor com o stick parado.
- **Filtro exponencial** de movimento (fator 0.60) — a posição se aproxima do alvo gradualmente, deixando o movimento da mesa macio.
- **Limitadores de curso por porcentagem** (`#define`) para ajustar o curso de cada servo à estrutura física sem forçar as engrenagens nos extremos.

### 📐 MPU6050 — `mpu6050.c`
- Barramento I²C em *fast-mode* (400 kHz); verifica `WHO_AM_I` antes de prosseguir.
- Leitura em **rajada de 14 bytes** (acelerômetro + temperatura + giroscópio).
- **Calibração do giroscópio** com 200 amostras na partida (executada dentro da task, sem travar o `app_main` nem disparar o *watchdog*).
- **Filtro complementar** (α = 0.96 @ 100 Hz) unindo a integração do giroscópio (rápida, sem *drift* de curto prazo) com a referência absoluta do acelerômetro.
- *Recalibração* dos ângulos sob demanda pelo botão do joystick.

### 💡 Status / LED — `status.c`
- Pisca o LED **5 vezes** ao final do boot e o mantém **aceso fixo** indicando que a mesa está pronta.
- Marca `system_ready` no estado global e imprime logs periódicos no monitor serial.

### 🧱 Estado comum — `system_common.c` / `.h`
- Define `system_data_t` (leituras do joystick, ângulos filtrados, limites de PWM, flags de calibração/vitória, *system_ready*) e o `data_mutex` compartilhado.

---

## 📡 Formato de telemetria (UART)

A cada ~100 ms (10 Hz) a `mpu6050_task` imprime uma linha JSON no monitor serial, pronta para ser consumida por um script no PC:

```json
{"pitch":-1.23,"roll":4.56,"ax":0.01,"ay":0.03,"az":0.99,"gx":0.00,"gy":0.01,"gz":0.00,"joy_x":2048,"joy_y":2048,"btn":0}
```

| Campo | Descrição |
|---|---|
| `pitch`, `roll` | Ângulos filtrados da mesa (graus) |
| `ax`, `ay`, `az` | Aceleração nos três eixos (g) |
| `gx`, `gy`, `gz` | Velocidade angular nos três eixos (°/s) |
| `joy_x`, `joy_y` | Leitura suavizada do joystick |
| `btn` | Estado do botão (0/1) |

---

## 🛠️ Como compilar e gravar

### Pré-requisitos
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) (v5.x recomendado)
- Toolchain configurado para o alvo **ESP32-C6**

### Hardware real
```bash
# Dentro da pasta do projeto
cd projeto-final

# Define o alvo (apenas na primeira vez)
idf.py set-target esp32c6

# Compila
idf.py build

# Grava e abre o monitor serial (ajuste a porta)
idf.py -p /dev/ttyUSB0 flash monitor
```

### Simulação no Wokwi
O arquivo `main/diagram.json` contém o circuito completo (joystick, servos, MPU6050 e LED). Basta abrir o projeto no [Wokwi](https://wokwi.com/) com o *builder* ESP-IDF, compilar e rodar a simulação.

---

## 🗺️ Roadmap das fases

- [x] **Fase 1 — Controle local da mesa**
  - [x] Leitura analógica dos eixos X e Y do joystick
  - [x] Conversão joystick → PWM para os dois servos
  - [x] Movimento suave e proporcional
  - [x] Organização em tasks FreeRTOS (joystick / servos / status)
- [x] **Fase 2 — Orientação e envio de dados**
  - [x] Leitura via I²C do acelerômetro e giroscópio
  - [x] Cálculo de pitch e roll com filtro complementar
  - [x] Envio periódico em JSON pela UART
  - [x] Task FreeRTOS exclusiva para o MPU6050
- [ ] **Fase 3 — Gêmeo digital e visualização**
  - [ ] Configurar InfluxDB + Grafana no computador
  - [ ] Script (Python/Node) para ler a serial e gravar no banco
  - [ ] Dashboard com pitch/roll em tempo real e gauge de orientação
  - [ ] Sincronização entre movimento físico e modelo digital

---

## 📂 Estrutura do repositório

```
projeto-final/
├── CMakeLists.txt
└── main/
    ├── CMakeLists.txt
    ├── diagram.json        # Circuito Wokwi (ESP32-C6 + periféricos)
    ├── main.c              # app_main: inicializa HW e cria as tasks
    ├── system_common.{c,h} # Estado global compartilhado + mutex
    ├── joystick.{c,h}      # Driver e task do joystick
    ├── servo.{c,h}         # Driver e task dos servos (LEDC PWM)
    ├── mpu6050.{c,h}       # Driver I²C, filtro complementar e telemetria
    └── status.{c,h}        # LED de status e logs
```

---

## 👥 Equipe

- **Fabrício Moreno** ([@FabsMS](https://github.com/FabsMS))
- **Maria Eduarda**
- **Luccas Daris**
- **Ynnayron Juan**

---

<p align="center"><sub>Projeto Final · Sistemas Embarcados · 2026.1</sub></p>
