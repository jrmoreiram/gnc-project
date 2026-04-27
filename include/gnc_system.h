#ifndef GNC_SYSTEM_H_
#define GNC_SYSTEM_H_

#include "flight_loop/flight_loop.h"

namespace gnc {

/**
 * @brief Classe de alto nível para inicializar e executar o sistema GNC.
 */
class GncSystem {
 public:
  void run();

 private:
  flight_loop::FlightLoop loop_;
};

}  // namespace gnc

#endif  // GNC_SYSTEM_H_
