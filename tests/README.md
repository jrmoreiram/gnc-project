# 🧪 GNC Project Tests

Este diretório contém todos os testes unitários e de integração para o projeto GNC (Guidance, Navigation & Control).

## 📁 Estrutura de Testes

```
/tests
├── CMakeLists.txt           # Configuração CMake para testes
├── run_tests.sh             # Script para executar testes
├── sensors_test.cpp         # Testes de sensores (IMU, GPS, Barômetro)
├── estimation_test.cpp      # Testes de estimação de estado (EKF)
├── control_test.cpp         # Testes de controladores (PID, Rate)
├── math_utils_test.cpp      # Testes de utilitários matemáticos
├── integration_test.cpp     # Testes de integração do sistema
└── README.md                # Este arquivo
```

## 📊 Cobertura de Testes

### 1. **Sensores** (sensors_test.cpp)
- ✅ IMU: Inicialização, dimensão de saída, validação de ruído
- ✅ GPS: Posição e velocidade válidas
- ✅ Barômetro: Altitude e pressão razoáveis

### 2. **Estimação de Estado** (estimation_test.cpp)
- ✅ EKF: Inicialização, vetor de estado, matriz de covariância
- ✅ State Estimator: Posição, velocidade, atitude válidas
- ✅ Covariância: Verificação de matriz positiva definida

### 3. **Controle** (control_test.cpp)
- ✅ PID: Resposta ao degrau, termos proporcionais, saturação, reset
- ✅ Rate Controller: Saída válida para diferentes entradas
- ✅ Actuator Controller: Saturação de comandos

### 4. **Utilitários Matemáticos** (math_utils_test.cpp)
- ✅ Operações vetoriais: Normalização, dot product, cross product
- ✅ Quaterniões: Normalização, conversão com Euler
- ✅ Matrizes: Inversa, SVD
- ✅ Ângulos: Wrapping, interpolação, saturação
- ✅ Funções utilitárias: Clamp, saturate

### 5. **Integração** (integration_test.cpp)
- ✅ Sistema GNC: Inicialização, execução sem exceções
- ✅ Fluxo de dados: Sensores → Estimador → Controle
- ✅ Progressão de estado: Verificação de mudanças de estado
- ✅ Estabilidade: Atitude e altitude dentro de limites

## 🚀 Como Executar os Testes

### Método 1: Script de Testes (Recomendado)

```bash
cd /home/jmmartins/Workspace/gnc-project/tests
./run_tests.sh
```

O script irá:
1. Verificar se o build existe, senão compila
2. Compilar os testes
3. Executar todos os testes
4. Mostrar resultados com cores

### Método 2: Compilação Manual + CTest

```bash
cd /home/jmmartins/Workspace/gnc-project
cmake -S . -B build
cmake --build build -j
cd build
ctest --output-on-failure -V
```

### Método 3: Executar Testes Diretamente

```bash
cd /home/jmmartins/Workspace/gnc-project/build/tests
./gnc_tests
```

### Opções de Execução Úteis

```bash
# Listar todos os testes disponíveis
./gnc_tests --gtest_list_tests

# Executar apenas testes de um módulo específico
./gnc_tests --gtest_filter="IMUTest.*"

# Executar com verbosidade aumentada
./gnc_tests --gtest_filter="*" -v

# Repetir testes 10 vezes
./gnc_tests --gtest_repeat=10
```

## 📋 Resultado Esperado

Ao executar os testes com sucesso, você verá:

```
Running main() from gmock_main.cc
[==========] Running XX tests from Y test suites.
[----------] Global test environment set-up.
[----------] Y tests from SensorTests
[ RUN      ] IMUTest.IMUInitialization
[       OK ] IMUTest.IMUInitialization (0 ms)
...
[==========] XX tests from Y test suites ran (XXX ms total).
[  PASSED  ] XX tests.
```

## 🔧 Dependências para Testes

- **Google Test (GTest)**: Framework de testes (baixado automaticamente)
- **Eigen3**: Operações lineares
- **YAML-CPP**: Carregamento de configurações
- **CMake 3.16+**: Sistema de build

## 📝 Adicionando Novos Testes

Para adicionar novos testes:

1. **Crie um novo arquivo de teste** seguindo a convenção `module_test.cpp`
2. **Adicione ao CMakeLists.txt**:
   ```cmake
   add_executable(gnc_tests
     ...
     novo_modulo_test.cpp
   )
   ```
3. **Estruture o teste com fixtures do Google Test**:
   ```cpp
   #include <gtest/gtest.h>
   
   class MyModuleTest : public ::testing::Test {
    protected:
     void SetUp() override { }
     void TearDown() override { }
   };
   
   TEST_F(MyModuleTest, TestName) {
     EXPECT_TRUE(condition);
   }
   ```

## 📚 Macros Úteis do Google Test

```cpp
// Assertions (falham o teste)
ASSERT_EQ(a, b)        // a == b
ASSERT_TRUE(condition)
ASSERT_FALSE(condition)
ASSERT_THROW(expr, exception)

// Expectations (continuam o teste)
EXPECT_EQ(a, b)
EXPECT_NEAR(a, b, tol) // Comparação com tolerância
EXPECT_TRUE(condition)
EXPECT_FALSE(condition)

// Com mensagens customizadas
EXPECT_EQ(a, b) << "Mensagem de erro personalizada";
```

## 🐛 Troubleshooting

### Erro: "Google Test not found"
```bash
# Se o download automático falhar, instale manualmente:
sudo apt-get install libgtest-dev
```

### Erro: "undefined reference to `main`"
```bash
# Certifique-se de que os sources estão inclusos no CMakeLists.txt
# e que GTest::gtest_main está linkado
```

### Testes falhando aleatoriamente
```bash
# Execute os testes múltiplas vezes para verificar flakiness:
./gnc_tests --gtest_repeat=10
```

## 📊 Cobertura de Testes

Para gerar relatório de cobertura (requer gcov/lcov):

```bash
# Compilar com flags de cobertura
cmake -S . -B build -DCMAKE_CXX_FLAGS="--coverage"
cmake --build build -j
ctest

# Gerar relatório
lcov --capture --directory . --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## 🎯 Próximas Melhorias

- [ ] Aumentar cobertura para > 80%
- [ ] Adicionar testes de performance
- [ ] Implementar benchmarks
- [ ] Adicionar testes de stress
- [ ] Integração contínua (CI/CD)

---

**Última atualização**: Maio 2025
**Framework de testes**: Google Test (GTest)
**Status**: ✅ Funcional e completo para testes básicos
