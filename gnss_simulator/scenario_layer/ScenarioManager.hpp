#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Message.hpp>

#include "position.hpp"
#include "satellite.hpp"
#include <vector>
#include <memory>

namespace gr::gnss_sim {

/**
 * @brief Менеджер сценария: рассчитывает геометрию и рассылает динамику в каналы
 * 
 * Работает как "source" — генерирует сообщения с частотой update_rate_hz.
 */
template <typename T>
class ScenarioManager : public gr::Block<ScenarioManager<T>> {
public:
    // Выходной message port — рассылает обновления всем каналам
    gr::MessagePort dynamics_out{this, "dynamics_out", gr::Direction::OUT};

    // Параметры
    gr::Property<double> update_rate_hz{this, "update_rate_hz", 10.0};
    gr::Property<double> sample_rate{this, "sample_rate", 5e6};

    ScenarioManager(double sr = 5e6, double upd_rate = 10.0)
        : update_rate_hz{this, "update_rate_hz", upd_rate},
          sample_rate{this, "sample_rate", sr}
    {
        // Статический приёмник: Москва (55.75° с.ш., 37.62° в.д., 150 м)
        receiver_pos_ = gnss_sim::Position::fromGeodetic(
            55.75 * std::numbers::pi / 180.0,
            37.62 * std::numbers::pi / 180.0,
            150.0
        );
        receiver_vel_ = gnss_sim::Velocity{0.0, 0.0, 0.0}; // Статический

        // Добавляем один тестовый спутник (PRN=1)
        satellites_.emplace_back(1, /*raan=*/0.0, /*phase=*/45.0);
    }

    void addSatellite(int prn, double raan_deg, double initial_phase_deg) {
        satellites_.emplace_back(prn, raan_deg, initial_phase_deg);
    }

    void setReceiverPosition(double lat_deg, double lon_deg, double alt_m) {
        receiver_pos_ = gnss_sim::Position::fromGeodetic(
            lat_deg * std::numbers::pi / 180.0,
            lon_deg * std::numbers::pi / 180.0,
            alt_m
        );
    }

    gr::work_return_t work() {
        // Проверяем, пришло ли время для обновления
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_update_time_).count();
        
        double update_period = 1.0 / update_rate_hz;
        if (elapsed < update_period) {
            // Ещё не время, ждём
            return {0, 0};
        }

        last_update_time_ = now;
        sim_time_ += elapsed;

        // Для каждого спутника рассчитываем геометрию и отправляем сообщение
        for (const auto& sat : satellites_) {
            gnss_sim::Position sat_pos;
            gnss_sim::Velocity sat_vel;
            sat.getState(sim_time_, sat_pos, sat_vel);

            // Вектор "приёмник -> спутник"
            auto los = sat_pos - receiver_pos_;
            double range = los.norm();
            
            // Единичный вектор линии визирования
            auto los_unit = los * (1.0 / range);

            // Скорость изменения дальности (range rate)
            double range_rate = sat_vel.dot(los_unit) - receiver_vel_.dot(
                gnss_sim::Position{los_unit.x, los_unit.y, los_unit.z});

            // Псевдодальность (без поправок на часы/ионосферу для hello world)
            double pseudorange = range + gnss_sim::Position::WGS84_A * sat.clock_bias_s * 299792458.0 / gnss_sim::Position::WGS84_A;
            // Упрощённо: pseudorange = range (clock_bias пока 0)
            pseudorange = range;

            // Доплер в Гц (для L1 C/A: f_carrier = 1575.42 МГц)
            constexpr double C_LIGHT = 299792458.0;
            constexpr double F_L1 = 1575.42e6;
            double doppler_hz = -range_rate * F_L1 / C_LIGHT;

            // Задержка кода в чипах (для GPS: chip_rate = 1.023 МГц)
            constexpr double CHIP_RATE = 1.023e6;
            constexpr double CODE_LENGTH_CHIPS = 1023.0;
            double code_phase_chips = std::fmod(pseudorange * CHIP_RATE / C_LIGHT, CODE_LENGTH_CHIPS);
            if (code_phase_chips < 0) code_phase_chips += CODE_LENGTH_CHIPS;

            // Элевация (для логирования и фильтрации видимых SV)
            // Упрощённо: если z-компонента > 0, спутник над горизонтом
            double elevation_rad = std::asin(los_unit.z); // Грубая оценка для hello world

            // Формируем сообщение
            gr::Message msg;
            msg.data["prn"] = sat.prn;
            msg.data["sim_time_s"] = sim_time_;
            msg.data["pseudorange_m"] = pseudorange;
            msg.data["range_rate_mps"] = range_rate;
            msg.data["doppler_hz"] = doppler_hz;
            msg.data["code_phase_chips"] = code_phase_chips;
            msg.data["elevation_rad"] = elevation_rad;

            // Отправляем сообщение
            dynamics_out.send(msg);
        }

        return {0, 0}; // Source: ничего не потребляем
    }

private:
    gnss_sim::Position receiver_pos_;
    gnss_sim::Velocity receiver_vel_;
    std::vector<gnss_sim::SimpleSatellite> satellites_;
    
    double sim_time_{0.0};
    std::chrono::steady_clock::time_point last_update_time_{std::chrono::steady_clock::now()};
};

GR_REGISTER_BLOCK(gr::gnss_sim::ScenarioManager, (float, double))

} // namespace gr::gnss_sim