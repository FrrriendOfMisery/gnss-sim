#pragma once

#include "core/common.hpp"
#include "core/position.hpp"

namespace gnss_sim {

/**
 * @brief Абстрактная орбитальная модель (стратегия)
 */
class OrbitModel {
public:
    virtual ~OrbitModel() = default;

    /// Позиция и скорость спутника в ECEF в момент time_sec (GPS время, с)
    virtual void getState(double time_sec, Position& pos, Velocity& vel) const = 0;

    virtual std::unique_ptr<OrbitModel> clone() const = 0;
};

} // namespace gnss_sim
