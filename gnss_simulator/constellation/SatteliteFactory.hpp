// include/gnss_sim/satellite_factories.hpp
#pragma once

#include "satellite.hpp"
#include "simple_circular_orbit.hpp"
#include "clock_model.hpp"

namespace gnss_sim {
namespace factories {

/**
 * @brief Создание GPS спутника
 */
inline Satellite createGpsSatellite(int prn, double raan_deg, double initial_phase_deg) {
    std::vector<SignalConfig> signals = {
        SignalConfigs::GPS_L1_CA(),
        SignalConfigs::GPS_L2C(),
        SignalConfigs::GPS_L5()
    };
    
    return Satellite(
        prn,
        "GPS",
        std::make_unique<SimpleCircularOrbit>(raan_deg, initial_phase_deg),
        std::make_unique<IdealClock>(),
        signals,
        "L1_CA"
    );
}

/**
 * @brief Создание GLONASS спутника (с FDMA)
 */
inline Satellite createGlonassSatellite(int slot, int freq_channel, 
                                        double raan_deg, double initial_phase_deg) {
    // Для GLONASS несущая зависит от частотного канала
    constexpr double F0 = 1602e6;
    constexpr double DFK = 0.5625e6;
    double carrier_l1 = F0 + freq_channel * DFK;
    double carrier_l2 = F0 * 1.5 + freq_channel * DFK * 1.5; // L2 = 1.5 * L1
    
    SignalConfig l1_of("L1_OF", carrier_l1, 0.511e6, 511.0);
    SignalConfig l2_of("L2_OF", carrier_l2, 0.511e6, 511.0);
    
    Satellite sat(
        slot,
        "GLONASS",
        std::make_unique<SimpleCircularOrbit>(raan_deg, initial_phase_deg, 25510e3, 64.8),
        std::make_unique<IdealClock>(),
        {l1_of, l2_of},
        "L1_OF"
    );
    
    sat.frequency_channel = freq_channel;
    return sat;
}

/**
 * @brief Создание Galileo спутника
 */
inline Satellite createGalileoSatellite(int prn, double raan_deg, double initial_phase_deg) {
    std::vector<SignalConfig> signals = {
        SignalConfigs::GAL_E1_OS(),
        SignalConfigs::GAL_E5A()
    };
    
    return Satellite(
        prn,
        "GALILEO",
        std::make_unique<SimpleCircularOrbit>(raan_deg, initial_phase_deg, 29600e3, 56.0),
        std::make_unique<IdealClock>(),
        signals,
        "E1_OS"
    );
}

/**
 * @brief Создание кастомного спутника (полная гибкость)
 */
inline Satellite createCustomSatellite(
    int id,
    const std::string& system_name,
    std::unique_ptr<OrbitModel> orbit,
    std::unique_ptr<ClockModel> clock,
    std::vector<SignalConfig> signals,
    const std::string& primary_signal)
{
    return Satellite(id, system_name, std::move(orbit), std::move(clock), 
                     std::move(signals), primary_signal);
}

} // namespace factories
} // namespace gnss_sim