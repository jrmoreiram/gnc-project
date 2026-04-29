# README.md

## Visão Geral

Este projeto é uma implementação de sistema de orientação, navegação e controle (GNC) para software de voo em C++. Inspirado pela arquitetura do Falcon 9 da SpaceX, visa fornecer uma solução robusta e eficiente para missões espaciais.

## Arquitetura utilizada

O sistema foi projetado utilizando uma arquitetura modular, permitindo fácil manutenção e escalabilidade. Os módulos principais incluem: controle de atitude, navegação inercial, e gerenciamento de missão, todos interconectados através de uma interface bem definida.

## Estrutura de diretórios

```
/gnc-project
    /src         # Diretório de código-fonte
    /include     # Cabeçalhos
    /tests       # Testes unitários
    /docs        # Documentação
```

## Módulos Principais

- `Controle de Atitude`: Responsável pela orientação da nave.
- `Navegação Inercial`: Processa dados de sensores para determinar a posição e velocidade.
- `Gerenciamento de Missão`: Coordena as atividades e objetivos da missão.

## Build e Run project

Para compilar o projeto, utilize os seguintes comandos:
```bash
mkdir build && cd build
cmake ..
make
```

Para executar:
```bash
./gnc_project_executable
```

## Testes

Os testes podem ser executados com:
```bash
cd tests
./run_tests.sh
```

## Ferramentas

- CMake - Para gerenciamento de construção.
- Google Test - Para testes unitários.
- Doxygen - Para geração de documentação.

## Convenções de Código

Seguir rigorosamente a norma C++11, e aplicar as convenções de estilo Google para nomes de variáveis e formatação.

## Roadmap

- [ ] Finalizar Módulo de Controle de Atitude
- [ ] Implementação de navegação por GPS
- [ ] Testes de integração

## Licença

Este projeto é licenciado sob a Licença MIT. Veja o arquivo LICENSE para mais detalhes.