#pragma once

#include "../core/common.hpp"
#include "../core/position.hpp"

namespace gnss_sim {

/**
 * @brief Кэшированное состояние спутника (обновляется каждый шаг сценария)
 */
struct SatelliteState {
    int id{0};
    Position position{};          // ECEF (м)
    Velocity velocity{};          // ECEF (м/с) — хранится как Position-вектор
    double elevation_rad{0.0};
    double azimuth_rad{0.0};
    double pseudorange_m{0.0};
    double range_rate_mps{0.0};
    double doppler_hz{0.0};
    double code_phase_chips{0.0};
    double clock_bias_s{0.0};
    bool is_visible{false};
};

} // namespace gnss_sim
