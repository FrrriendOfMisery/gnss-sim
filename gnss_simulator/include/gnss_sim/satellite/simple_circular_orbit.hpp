#pragma once

#include "../core/common.hpp"
#include "../core/position.hpp"
#include "orbit_model.hpp"

#include <numbers>

namespace gnss_sim {

/**
 * @brief Упрощённая круговая орбита (для hello-world и builtin-эфемерид)
 *
 * Спутник движется по круговой орбите с заданным RAAN и начальной фазой.
 * Параметры по умолчанию соответствуют GPS MEO (a ~ 26560 км, i = 55°).
 */
class SimpleCircularOrbit : public OrbitModel {
public:
    SimpleCircularOrbit(double raan_deg = 0.0,
                        double initial_phase_deg = 0.0,
                        double radius_m = 26560e3,
                        double inclination_deg = 55.0)
        : raan_rad_(raan_deg * std::numbers::pi / 180.0),
          phase0_rad_(initial_phase_deg * std::numbers::pi / 180.0),
          radius_m_(radius_m),
          incl_rad_(inclination_deg * std::numbers::pi / 180.0)
    {
        // Угловая скорость из третьего закона Кеплера: n = sqrt(GM / a^3)
        constexpr double GM_EARTH = 3.986004418e14; // м^3/с^2
        mean_motion_ = std::sqrt(GM_EARTH / (radius_m_ * radius_m_ * radius_m_));
    }

    void getState(double time_sec, Position& pos, Velocity& vel) const override {
        const double u = phase0_rad_ + mean_motion_ * time_sec; // аргумент широты
        const double cos_u = std::cos(u), sin_u = std::sin(u);
        const double cos_i = std::cos(incl_rad_), sin_i = std::sin(incl_rad_);
        const double cos_r = std::cos(raan_rad_), sin_r = std::sin(raan_rad_);

        // Позиция в плоскости орбиты, затем повороты на наклонение и RAAN
        const double x_orb = radius_m_ * cos_u;
        const double y_orb = radius_m_ * sin_u;

        pos.x = cos_r * x_orb - sin_r * cos_i * y_orb;
        pos.y = sin_r * x_orb + cos_r * cos_i * y_orb;
        pos.z = sin_i * y_orb;

        const double vx_orb = -radius_m_ * mean_motion_ * sin_u;
        const double vy_orb =  radius_m_ * mean_motion_ * cos_u;

        vel.vx = cos_r * vx_orb - sin_r * cos_i * vy_orb;
        vel.vy = sin_r * vx_orb + cos_r * cos_i * vy_orb;
        vel.vz = sin_i * vy_orb;
    }

    std::unique_ptr<OrbitModel> clone() const override {
        return std::make_unique<SimpleCircularOrbit>(*this);
    }

private:
    double raan_rad_;
    double phase0_rad_;
    double radius_m_;
    double incl_rad_;
    double mean_motion_{0.0}; // рад/с
};

} // namespace gnss_sim
