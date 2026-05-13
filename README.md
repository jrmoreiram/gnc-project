## 🚀 Visão Geral

Este projeto é uma implementação completa de um sistema de **Orientação, Navegação e Controle (GNC)** para simulação de voo aeroespacial em C++. Inspirado pela arquitetura do Falcon 9 da SpaceX, o sistema simula o comportamento de uma nave espacial durante uma missão, integrando sensores, estimação de estado, navegação e controle em um loop de simulação coerente.

## 🏗️ Arquitetura utilizada

O sistema foi projetado utilizando uma **arquitetura modular e em camadas**, permitindo fácil manutenção, escalabilidade e testabilidade. Todos os módulos são interconectados através de interfaces bem definidas e executados em um loop de simulação síncrono que roda a 100 Hz (período de 0.01s).

## 📁 Estrutura de diretórios

```
/gnc-project
    /src
        /sensors           # Módulos de sensores simulados (IMU, GPS, Barômetro)
        /estimation        # Estimação de estado (EKF, State Estimator)
        /guidance          # Navegação (Trajectory Planner, Guidance Law)
        /control           # Controle (PID, Rate Controller, Actuator)
        /flight_loop       # Loop de simulação central
        /utils             # Utilitários (Math, Logger, Config Loader)
    /include               # Headers (.h)
    /build                 # Diretório de compilação
    /config                # Arquivos de configuração
    /docs                  # Documentação
		/tests
    /flight_log.csv        # Arquivo de log com dados de voo simulados
```

## 🧩 Módulos Principais

### 1. **Sensores** 📡
- **IMU** (Inertial Measurement Unit): Simula aceleração linear e velocidade angular
- **GPS**: Fornece posição e velocidade global da nave
- **Barômetro**: Mede altitude e pressão atmosférica

### 2. **Estimação de Estado** 🧮
- **EKF** (Extended Kalman Filter): Funde dados dos sensores para estimar o estado real da nave de forma robusta
- **State Estimator**: Mantém atualizado o estado completo da aeronave (posição, velocidade, atitude)

### 3. **Navegação** 🗺️
- **Trajectory Planner**: Planeja trajetórias desejadas para a missão
- **Guidance Law**: Calcula comandos de guiagem para seguir a trajetória planejada

### 4. **Controle** ✈️
- **PID Controller**: Implementa controladores PID para estabilização do sistema
- **Rate Controller**: Controla as taxas de rotação da nave
- **Actuator Controller**: Gerencia os atuadores da nave (motores, superfícies de controle)

### 5. **Loop de Voo** 🔄
- **Flight Loop**: Integra todos os módulos em uma simulação temporal coerente que executa por 90 segundos com passo de 0.01s

### 6. **Utilitários** 🛠️
- **Math Utils**: Operações matemáticas avançadas (álgebra linear, trigonometria)
- **Data Logger**: Registra dados da simulação para análise posterior
- **Config Loader**: Carrega configurações do sistema a partir de arquivos YAML

## ⚙️ Pré-requisitos

Para compilar e executar o projeto, você precisa instalar as seguintes dependências:

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install cmake g++ libeigen3-dev libyaml-cpp-dev
```

**macOS (com Homebrew):**
```bash
brew install cmake eigen yaml-cpp
```

## 🏗️ Build e Run project

### Compilar o projeto:
```bash
cd /home/jmmartins/Workspace/gnc-project
cmake -S . -B build          # Gera arquivos de configuração CMake
cmake --build build -j       # Compila usando todos os cores disponíveis
```

### Executar a simulação:
```bash
./build/gnc_app
```

**Resultado esperado:**
- A simulação GNC rodará por 90 segundos com passo de integração de 0.01s (100 Hz)
- Mensagem de sucesso: `"Simulação GNC concluída com sucesso."`
- Dados de voo são registrados no arquivo `flight_log.csv` para análise posterior

### Limpando arquivos de build:
```bash
rm -rf build/
```

## 🧪 Testes

Os testes podem ser executados com:
```bash
cd tests
./run_tests.sh
```

## 🛠️ Ferramentas e Dependências

- **CMake 3.16+** - Sistema de gerenciamento de construção
- **C++17** - Padrão moderno da linguagem C++
- **Eigen3** - Biblioteca de álgebra linear para operações com matrizes e vetores
- **YAML-CPP** - Carregamento e processamento de arquivos de configuração YAML
- **Google Test** - Framework para testes unitários (opcional, para testes)
- **Doxygen** - Geração de documentação (opcional)

## 📐 Convenções de Código

- **Padrão:** C++17 com conformidade estrita
- **Estilo:** Convenções do Google C++ Style Guide
  - Nomes de variáveis em snake_case: `flight_speed`
  - Nomes de classes em PascalCase: `GncSystem`, `EKFilter`
  - Constantes em UPPER_SNAKE_CASE: `MAX_ALTITUDE`
  - Métodos privados com prefixo: `private_method()`
- **Formatação:** Indentação de 2 espaços, linhas máximo 100 caracteres
- **Documentação:** Comentários em português, seguindo padrão Doxygen

## 🔭 Roadmap

- [x] Arquitetura modular base implementada
- [x] Sensores simulados (IMU, GPS, Barômetro)
- [x] Estimador de estado com EKF
- [x] Controlador PID
- [ ] Finalizar otimizações do Módulo de Controle de Atitude
- [ ] Implementar navegação por GPS com maior precisão
- [ ] Testes de integração completos
- [ ] Validação contra dados de voo reais
- [ ] Interface gráfica de visualização em tempo real

## 📚 Referências

- Wie, B. — *Space Vehicle Dynamics and Control* (AIAA, 2008)
- Stevens, B. L. — *Aircraft Control and Simulation* (Wiley, 2015)
- Zarchan, P. — *Tactical and Strategic Missile Guidance* (AIAA, 2012)
- SpaceX AMA — Reddit r/spacex (2015)

## 📄 Licença

Este projeto é licenciado sob a Licença MIT. Veja o arquivo LICENSE para mais detalhes.

## 📌 Nota
> Este projeto é educacional e de simulação implementado por um desenvolvedor👨‍💻 entusiasta aeroespacial🚀.
> Sistemas de voo reais exigem certificação, validação formal e hardware dedicado.
