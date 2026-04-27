# Flight Software GNC — C++

> Sistema de Guiagem, Navegação e Controle (GNC) para software de voo embarcado em C++.
> Inspirado na arquitetura utilizada em foguetes modernos como o Falcon 9 da SpaceX.

* * *

![dd168576bc23407991d88bb8cc39d7a2](file:///C:/Users/jmmartins/OneDrive%20-%20Neogrid/Imagens/Typedown/dd168576-bc23-4079-91d8-8bb8cc39d7a2.png?msec=1777316880919)

---

## 🚀 Visão Geral

Este projeto implementa um sistema GNC completo em C++ com ciclo de controle de **100 Hz**, cobrindo as camadas de:

- **Navigation** — Filtro de Kalman Estendido (EKF) com fusão IMU + GPS
- **Guidance** — Gerador de trajetória com interpolação de waypoints
- **Control** — Controlador PID de atitude (roll / pitch / yaw) com TVC
- **Flight Loop** — Orquestração do ciclo de controle principal

---

## 🏗️ Arquitetura GNC

```
SENSORS (IMU / GPS / Star Tracker)
         │
         ▼
NAVIGATION (Estimação de Estado)
  └─ ExtendedKalmanFilter (EKF)
  └─ Fusão de Sensores
  └─ Posição / Velocidade / Atitude
         │
         ▼
GUIDANCE (Planejamento de Trajetória)
  └─ GuidanceSystem
  └─ Waypoints + Interpolação Linear
  └─ Perfil de Velocidade / Altitude
         │
         ▼
CONTROL (Controlador PID / LQR)
  └─ AttitudeController
  └─ PIDController (roll, pitch, yaw)
  └─ TVC (Thrust Vector Control)
         │
         ▼
ACTUATORS (Motores / TVC / RCS)
  └─ Gimbal do Motor
  └─ Propulsores RCS
         │
         └──────────────────► Feedback Loop
```

---

## 📁 Estrutura de Diretórios

```
flight-software/
├── CMakeLists.txt
├── README.md
│
├── include/
│   ├── core/
│   │   ├── vector3.h            # struct Vector3
│   │   ├── vehicle_state.h      # struct VehicleState
│   │   └── types.h
│   ├── gnc/
│   │   ├── navigation/
│   │   │   └── ekf.h            # ExtendedKalmanFilter
│   │   ├── guidance/
│   │   │   ├── waypoint.h       # struct Waypoint
│   │   │   └── guidance.h       # GuidanceSystem
│   │   └── control/
│   │       ├── pid_controller.h # PIDController
│   │       └── attitude_ctrl.h  # AttitudeController
│   └── flight/
│       └── flight_software.h
│
├── src/
│   ├── main.cpp
│   ├── core/
│   │   ├── vector3.cpp
│   │   └── vehicle_state.cpp
│   ├── gnc/
│   │   ├── navigation/
│   │   │   └── ekf.cpp
│   │   ├── guidance/
│   │   │   ├── waypoint.cpp
│   │   │   └── guidance.cpp
│   │   └── control/
│   │       ├── pid_controller.cpp
│   │       └── attitude_ctrl.cpp
│   └── flight/
│       └── flight_software.cpp
│
├── simulation/
│   ├── sim_sensors.cpp
│   ├── sim_vehicle.cpp
│   └── world_model.h
│
├── tests/
│   ├── test_ekf.cpp
│   ├── test_guidance.cpp
│   ├── test_pid.cpp
│   └── test_flight_loop.cpp
│
├── config/
│   └── mission.yaml
│
└── tools/
    └── plot_telemetry.py
```

---

## 🧩 Módulos Principais

### `core/` — Tipos e Matemática Base

| Arquivo | Conteúdo |
|---|---|
| `vector3.h` | `struct Vector3` com operações aritméticas, norma e normalização |
| `vehicle_state.h` | `struct VehicleState` com posição, velocidade, atitude, massa e tempo |

### `gnc/navigation/` — Estimação de Estado

| Arquivo | Conteúdo |
|---|---|
| `ekf.h / ekf.cpp` | `ExtendedKalmanFilter` — predição via IMU + atualização via GPS |

**Ciclo EKF:**
1. `predict(accel, dt)` — integra dinâmica com aceleração medida
2. `updateGPS(pos)` — corrige estimativa com medição GPS
3. `getPosition()` / `getVelocity()` — retorna estado estimado

### `gnc/guidance/` — Planejamento de Trajetória

| Arquivo | Conteúdo |
|---|---|
| `waypoint.h` | `struct Waypoint` com posição, velocidade e tempo |
| `guidance.h / guidance.cpp` | `GuidanceSystem` com interpolação linear entre waypoints |

### `gnc/control/` — Controle de Atitude

| Arquivo | Conteúdo |
|---|---|
| `pid_controller.h / .cpp` | `PIDController` com anti-windup e saturação de saída |
| `attitude_ctrl.h / .cpp` | `AttitudeController` com PIDs independentes para roll, pitch e yaw |

**Ganhos padrão:**

| Eixo | Kp | Ki | Kd |
|---|---|---|---|
| Roll | 2.0 | 0.1 | 0.5 |
| Pitch | 2.5 | 0.1 | 0.6 |
| Yaw | 1.5 | 0.05 | 0.4 |

### `flight/` — Loop Principal

| Arquivo | Conteúdo |
|---|---|
| `flight_software.h / .cpp` | `FlightSoftware` — orquestra EKF + Guidance + Control a 100 Hz |

**Ciclo de controle (100 Hz / 10ms):**
```
1. readIMU()       → ekf.predict()
2. readGPS()       → ekf.updateGPS()
3. guidance.getReference(t)
4. attCtrl.compute(ref, state, dt)
5. sendTVCCommand(cmd)
6. logTelemetry()
```

---

## ⚙️ Build

### Requisitos

- CMake >= 3.16
- GCC >= 11 ou Clang >= 13
- C++17 ou superior
- Google Test (para testes unitários)

### Compilar

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Executar simulação

```bash
./flight_software
```

### Rodar testes

```bash
ctest --output-on-failure
```

---

## 🧪 Testes

| Arquivo | O que testa |
|---|---|
| `test_ekf.cpp` | Convergência do filtro com ruído sintético |
| `test_pid.cpp` | Resposta ao degrau, overshoot e tempo de assentamento |
| `test_guidance.cpp` | Interpolação entre waypoints e extrapolação |
| `test_flight_loop.cpp` | Integração completa do ciclo GNC |

---

## 🛠️ Ferramentas

### `tools/plot_telemetry.py`

Script Python para visualizar os dados de telemetria gerados pela simulação:

```bash
python3 tools/plot_telemetry.py logs/telemetry.csv
```

Gera gráficos de:
- Posição (x, y, z) ao longo do tempo
- Velocidade e aceleração
- Atitude (roll, pitch, yaw)
- Comandos TVC

---

## 📐 Convenções de Código

- **Padrão:** C++17
- **Formatação:** `.clang-format` (Google Style)
- **Análise estática:** `.clang-tidy`
- **Documentação:** Doxygen (`/** */`)
- **Unidades:** SI — metros, segundos, radianos, kg

---

## 🔭 Roadmap

- [ ] Implementar controlador LQR como alternativa ao PID
- [ ] Adicionar modelo de perturbação atmosférica
- [ ] Integrar com RTOS (FreeRTOS / VxWorks)
- [ ] Suporte a múltiplos computadores de voo (redundância)
- [ ] Implementar MPC (Model Predictive Control)
- [ ] Adicionar módulo de detecção e isolamento de falhas (FDI)

---

## 📚 Referências

- Wie, B. — *Space Vehicle Dynamics and Control* (AIAA, 2008)
- Stevens, B. L. — *Aircraft Control and Simulation* (Wiley, 2015)
- Zarchan, P. — *Tactical and Strategic Missile Guidance* (AIAA, 2012)
- SpaceX AMA — Reddit r/spacex (2015)

---

## 📄 Licença

MIT License — veja `LICENSE` para detalhes.

---

> **Nota:** Este projeto é educacional e de simulação.
> Sistemas de voo reais exigem certificação, validação formal e hardware dedicado.