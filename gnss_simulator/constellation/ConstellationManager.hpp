#pragma once

#include "satellite.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace gnss_sim {

class ConstellationManager {
public:
    explicit ConstellationManager(double elevation_mask_deg = 5.0, size_t max_channels = 12)
        : elevation_mask_rad_(elevation_mask_deg * std::numbers::pi / 180.0),
          max_channels_(max_channels)
    {}

    void addSatellite(Satellite sat) {
        satellites_.push_back(std::move(sat));
    }

    /**
     * @brief Обновление видимости
     */
    std::vector<Satellite*> updateVisibility(
        const Position& receiver_pos,
        const Velocity& receiver_vel,
        double time_sec)
    {
        // Обновляем все спутники
        for (auto& sat : satellites_) {
            sat.update(receiver_pos, receiver_vel, time_sec);
        }

        // Собираем видимые
        std::vector<Satellite*> visible;
        visible.reserve(satellites_.size());
        
        for (auto& sat : satellites_) {
            if (sat.is_healthy && 
                sat.state.is_visible && 
                sat.state.elevation_rad >= elevation_mask_rad_) {
                visible.push_back(&sat);
            }
        }

        // Сортируем по элевации
        std::sort(visible.begin(), visible.end(),
            [](const Satellite* a, const Satellite* b) {
                return a->state.elevation_rad > b->state.elevation_rad;
            });

        // Ограничиваем
        if (visible.size() > max_channels_) {
            visible.resize(max_channels_);
        }

        return visible;
    }

    [[nodiscard]] std::vector<Satellite>& getAllSatellites() {
        return satellites_;
    }

    void setElevationMask(double elevation_deg) {
        elevation_mask_rad_ = elevation_deg * std::numbers::pi / 180.0;
    }

    void setMaxChannels(size_t max_ch) {
        max_channels_ = max_ch;
    }

private:
    std::vector<Satellite> satellites_;
    double elevation_mask_rad_;
    size_t max_channels_;
};

} // namespace gnss_sim