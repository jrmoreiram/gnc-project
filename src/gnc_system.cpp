#include "gnc_system.h"

namespace gnc {

void GncSystem::run() {
  // Executa uma simulação padrão. Em produção este valor viria de YAML.
  loop_.run(90.0, 0.01);
}

}  // namespace gnc
