#include <exception>
#include <iostream>

#include "gnc_system.h"

int main() {
  try {
    gnc::GncSystem system;
    system.run();
    std::cout << "Simulação GNC concluída com sucesso." << std::endl;
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Falha no sistema GNC: " << ex.what() << std::endl;
    return 1;
  }
}
