#pragma once

#include <string>
#include <vector>
#include <map>

namespace gnss_sim {

/**
 * @brief Конфигурация одного сигнала спутника
 * 
 * Описывает все параметры конкретного сигнала (L1 C/A, E1 OS, и т.д.)
 */
struct SignalConfig {
    std::string name;                    // Имя сигнала (например, "L1_CA", "E1_OS")
    double carrier_frequency_hz{0.0};    // Несущая частота (Гц)
    double code_chip_rate_hz{0.0};       // Частота чипов кода (Гц)
    double code_length_chips{0.0};       // Длина кода (чипов)
    double code_period_sec{0.0};         // Период кода (сек) = code_length / chip_rate
    
    // Модуляция (для будущего использования)
    enum class Modulation {
        BPSK,
        QPSK,
        BOC,
        CBOC,
        ALTBOC
    };
    Modulation modulation{Modulation::BPSK};
    
    // Уровни мощности (относительные)
    double relative_power_db{0.0};       // Относительная мощность (дБ)
    
    SignalConfig() = default;
    
    SignalConfig(std::string name_, double carrier_hz, double chip_rate_hz, double code_length)
        : name(std::move(name_)),
          carrier_frequency_hz(carrier_hz),
          code_chip_rate_hz(chip_rate_hz),
          code_length_chips(code_length)
    {
        code_period_sec = code_length / chip_rate_hz;
    }
    
    [[nodiscard]] double getCodePeriod() const {
        return code_length_chips / code_chip_rate_hz;
    }
};

/**
 * @brief Предопределённые конфигурации сигналов (для удобства)
 * 
 * Это НЕ часть класса Satellite, а просто helper-функции для быстрого создания.
 */
namespace SignalConfigs {
    inline SignalConfig GPS_L1_CA() {
        return SignalConfig("L1_CA", 1575.42e6, 1.023e6, 1023.0);
    }
    
    inline SignalConfig GPS_L2C() {
        return SignalConfig("L2C", 1227.60e6, 1.023e6, 10230.0);
    }
    
    inline SignalConfig GPS_L5() {
        return SignalConfig("L5", 1176.45e6, 10.23e6, 10230.0);
    }
    
    inline SignalConfig GAL_E1_OS() {
        return SignalConfig("E1_OS", 1575.42e6, 1.023e6, 4092.0);
    }
    
    inline SignalConfig GAL_E5A() {
        return SignalConfig("E5A", 1176.45e6, 10.23e6, 10230.0);
    }
    
    inline SignalConfig GLO_L1_OF() {
        return SignalConfig("L1_OF", 1602e6, 0.511e6, 511.0); // Базовая частота, FDMA через параметр
    }
    
    inline SignalConfig BDS_B1I() {
        return SignalConfig("B1I", 1561.098e6, 2.046e6, 2046.0);
    }
}

} // namespace gnss_sim