#pragma once

#include "../core/common.hpp"
#include "../core/position.hpp"
#include "satellite_state.hpp"
#include "orbit_model.hpp"
#include "clock_model.hpp"
#include "signal_config.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <numbers>
#include <cmath>
#include <optional>

namespace gnss_sim {

/**
 * @brief Универсальный класс спутника — полностью независим от конкретной ГНСС
 * 
 * Все параметры (орбита, часы, сигналы) передаются через модели и конфигурации.
 * Может представлять спутник любой констелляции: GPS, GLONASS, Galileo, BeiDou, QZSS, или даже кастомной.
 */
class Satellite {
public:
    // =========================================================================
    // Идентификация
    // =========================================================================
    int id{0};                                    // Уникальный ID (PRN, slot, и т.д.)
    std::string system_name{"UNKNOWN"};           // Имя системы (например, "GPS", "GALILEO")
    std::string name{""};                         // Полное имя (например, "GPS PRN 1")
    
    // Дополнительные параметры (для специфичных систем)
    int frequency_channel{0};                     // Для FDMA (GLONASS)
    std::map<std::string, double> extra_params;   // Любые дополнительные параметры

    // =========================================================================
    // Характеристики
    // =========================================================================
    bool is_healthy{true};                        // Статус здоровья
    double ura_m{0.0};                            // User Range Accuracy (м)

    // =========================================================================
    // Модели (стратегии)
    // =========================================================================
    std::unique_ptr<OrbitModel> orbit_model;
    std::unique_ptr<ClockModel> clock_model;

    // =========================================================================
    // Конфигурация сигналов
    // =========================================================================
    std::vector<SignalConfig> signals;            // Все поддерживаемые сигналы
    std::string primary_signal_name{""};          // Имя основного сигнала (для расчётов)

    // =========================================================================
    // Кэшированное состояние (обновляется в update())
    // =========================================================================
    SatelliteState state;

    // =========================================================================
    // Конструкторы
    // =========================================================================

    Satellite() = default;

    /**
     * @brief Универсальный конструктор
     * @param id_ Уникальный ID спутника
     * @param system_name_ Имя системы
     * @param orbit Орбитальная модель
     * @param clock Модель часов
     * @param signals_ Список поддерживаемых сигналов
     * @param primary_signal Имя основного сигнала (для расчётов доплера, задержки кода)
     */
    Satellite(int id_,
              std::string system_name_,
              std::unique_ptr<OrbitModel> orbit,
              std::unique_ptr<ClockModel> clock,
              std::vector<SignalConfig> signals_,
              std::string primary_signal = "")
        : id(id_),
          system_name(std::move(system_name_)),
          orbit_model(std::move(orbit)),
          clock_model(std::move(clock)),
          signals(std::move(signals_))
    {
        name = system_name + " " + std::to_string(id);
        
        // Если основной сигнал не указан, берём первый
        if (primary_signal.empty() && !signals.empty()) {
            primary_signal_name = signals[0].name;
        } else {
            primary_signal_name = std::move(primary_signal);
        }
    }

    // =========================================================================
    // Copy/Move семантика (из-за unique_ptr)
    // =========================================================================
    
    Satellite(const Satellite& other)
        : id(other.id),
          system_name(other.system_name),
          name(other.name),
          frequency_channel(other.frequency_channel),
          extra_params(other.extra_params),
          is_healthy(other.is_healthy),
          ura_m(other.ura_m),
          orbit_model(other.orbit_model ? other.orbit_model->clone() : nullptr),
          clock_model(other.clock_model ? other.clock_model->clone() : nullptr),
          signals(other.signals),
          primary_signal_name(other.primary_signal_name),
          state(other.state)
    {}

    Satellite& operator=(const Satellite& other) {
        if (this != &other) {
            Satellite tmp(other);
            std::swap(id, tmp.id);
            std::swap(system_name, tmp.system_name);
            std::swap(name, tmp.name);
            std::swap(frequency_channel, tmp.frequency_channel);
            std::swap(extra_params, tmp.extra_params);
            std::swap(is_healthy, tmp.is_healthy);
            std::swap(ura_m, tmp.ura_m);
            std::swap(orbit_model, tmp.orbit_model);
            std::swap(clock_model, tmp.clock_model);
            std::swap(signals, tmp.signals);
            std::swap(primary_signal_name, tmp.primary_signal_name);
            std::swap(state, tmp.state);
        }
        return *this;
    }

    Satellite(Satellite&&) noexcept = default;
    Satellite& operator=(Satellite&&) noexcept = default;

    // =========================================================================
    // Обновление состояния
    // =========================================================================

    /**
     * @brief Обновление состояния спутника
     */
    void update(const Position& receiver_pos, const Velocity& receiver_vel, double time_sec) {
        state.id = id;

        // 1. Позиция и скорость спутника
        if (!orbit_model) {
            state.is_visible = false;
            return;
        }
        orbit_model->getState(time_sec, state.position, state.velocity);

        // 2. Геометрия
        auto los = state.position - receiver_pos;
        double geometric_range = los.norm();
        
        if (geometric_range < 1.0) {
            state.is_visible = false;
            return;
        }

        auto los_unit = los * (1.0 / geometric_range);

        // 3. Элевация
        auto receiver_unit = receiver_pos * (1.0 / receiver_pos.norm());
        double sin_elev = los_unit.dot(receiver_unit);
        state.elevation_rad = std::asin(sin_elev);

        // 4. Азимут
        state.azimuth_rad = std::atan2(los_unit.y, los_unit.x);

        // 5. Range rate
        state.range_rate_mps = state.velocity.dot(los_unit) - 
                               receiver_vel.dot(Position{los_unit.x, los_unit.y, los_unit.z});

        // 6. Поправка часов
        double clock_bias_s = clock_model ? clock_model->getBias(time_sec) : 0.0;
        constexpr double C_LIGHT = 299792458.0;

        // 7. Псевдодальность
        state.pseudorange_m = geometric_range + C_LIGHT * clock_bias_s;

        // 8. Параметры основного сигнала
        auto primary_sig = getSignal(primary_signal_name);
        if (!primary_sig) {
            state.is_visible = false;
            return;
        }

        // 9. Доплер (с учётом дрейфа часов)
        state.doppler_hz = -state.range_rate_mps * primary_sig->carrier_frequency_hz / C_LIGHT;
        if (clock_model) {
            state.doppler_hz *= (1.0 + clock_model->getDrift(time_sec));
        }

        // 10. Задержка кода в чипах
        state.code_phase_chips = std::fmod(
            state.pseudorange_m * primary_sig->code_chip_rate_hz / C_LIGHT,
            primary_sig->code_length_chips
        );
        if (state.code_phase_chips < 0) {
            state.code_phase_chips += primary_sig->code_length_chips;
        }

        // 11. Видимость
        state.is_visible = (state.elevation_rad > 0.0);
    }

    // =========================================================================
    // Работа с сигналами
    // =========================================================================

    /**
     * @brief Получение конфигурации сигнала по имени
     */
    [[nodiscard]] std::optional<SignalConfig> getSignal(const std::string& signal_name) const {
        for (const auto& sig : signals) {
            if (sig.name == signal_name) {
                return sig;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief Добавление сигнала
     */
    void addSignal(const SignalConfig& signal) {
        signals.push_back(signal);
    }

    /**
     * @brief Проверка наличия сигнала
     */
    [[nodiscard]] bool hasSignal(const std::string& signal_name) const {
        return getSignal(signal_name).has_value();
    }
};

} // namespace gnss_sim
