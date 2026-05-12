# 🧪 Guia de Testes - GNC Project

## ⚡ Quick Start

```bash
# Clonar e navegar
cd /home/jmmartins/Workspace/gnc-project

# Executar testes (opção mais fácil)
./tests/run_tests.sh

# Ou compilar e testar manualmente
cmake -S . -B build && cmake --build build -j
cd build && ctest --output-on-failure
```

## 📊 O que há nos testes?

### 49 Testes Estruturados em 5 Módulos:

| Módulo | Arquivo | Testes | Cobertura |
|--------|---------|--------|-----------|
| **Sensores** | `sensors_test.cpp` | 8 | IMU, GPS, Barômetro |
| **Estimação** | `estimation_test.cpp` | 8 | EKF, State Estimator |
| **Controle** | `control_test.cpp` | 10 | PID, Rate, Actuator |
| **Matemática** | `math_utils_test.cpp` | 15 | Vetores, Quaterniões, Matrizes |
| **Integração** | `integration_test.cpp` | 8 | Sistema Completo |

## 🎯 Exemplos de Uso

### Rodar todos os testes
```bash
./tests/run_tests.sh
```

### Rodar apenas testes de sensores
```bash
./build/tests/gnc_tests --gtest_filter="*Sensor*"
```

### Listar todos os testes disponíveis
```bash
./build/tests/gnc_tests --gtest_list_tests
```

### Rodar testes com verbosidade
```bash
./build/tests/gnc_tests --gtest_filter="*" -v
```

## 📚 Documentação Detalhada

Para informações completas sobre testes, veja:
- [`tests/README.md`](tests/README.md) - Guia completo de testes
- [`README.md`](README.md) - Documentação geral do projeto

## ✅ Status dos Testes

- ✅ Framework: Google Test (GTest)
- ✅ Cobertura: Sensores, Estimação, Controle, Matemática, Integração
- ✅ Execução: Automatizada via CMake/CTest
- ✅ Documentação: Completa e atualizada

---
**Última atualização**: Maio 2025
