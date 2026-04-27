#ifndef GNC_UTILS_MATH_UTILS_H_
#define GNC_UTILS_MATH_UTILS_H_

namespace gnc::utils {

/**
 * @brief Limita valor em intervalo [min, max].
 */
double clamp(double value, double min_v, double max_v);

/**
 * @brief Normaliza ângulo para intervalo [-pi, pi].
 */
double wrapAngle(double angle_rad);

}  // namespace gnc::utils

#endif  // GNC_UTILS_MATH_UTILS_H_
