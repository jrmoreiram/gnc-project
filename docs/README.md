# GNC Project (Flight Guidance, Navigation and Control)

Projeto C++ com arquitetura modular de GNC para simulação de voo, contendo:

- **Sensoriamento**: IMU, GPS e Barômetro
- **Navegação/Estimativa**: EKF simplificado para posição, velocidade e atitude
- **Guiamento**: lei de guiamento com planejamento por waypoints
- **Controle**: malha externa (altitude/heading/speed) + malha interna de taxas + atuadores
- **Flight Loop**: sequência `Sense -> Estimate -> Guide -> Control -> Propagate`
- **Utilitários**: logger CSV, utilidades matemáticas e loader de YAML

## Estrutura

- `include/`: interfaces e tipos
- `src/`: implementações
- `config/`: parâmetros de veículo, GNC e missão
- `docs/`: documentação

## Dependências

- CMake >= 3.16
- C++17
- Eigen3
- yaml-cpp

No Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libyaml-cpp-dev
```

## Build e execução

```bash
cd /home/ubuntu/gnc-project
cmake -S . -B build
cmake --build build -j
./build/gnc_app
```

Ao executar, um arquivo `flight_log.csv` é gerado na raiz do projeto.

## Observações

- O modelo dinâmico está simplificado para foco na arquitetura GNC.
- O EKF implementa etapas reais de predição/correção com matrizes de covariância.
- Os ganhos PID podem ser ajustados via `config/gnc_config.yaml` para calibração.
