#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Property.hpp>

#include <complex>
#include <span>
#include <vector>

// Подключаем наши оптимизированные алгоритмы
#include "multipath_code_generator.hpp"
#include "nco.hpp"

namespace gr::gnss_sim {

/**
 * @brief Блок канала GPS L1 C/A для GNU Radio 4
 *
 * Генерирует сигнал одного спутника с поддержкой многолучевости,
 * доплеровского сдвига и модуляции навигационными данными.
 *
 * @tparam T Тип данных сэмпла (float или double)
 */
template <typename T>
class GpsChannelBlock : public gr::Block<GpsChannelBlock<T>> {
public:
    // =========================================================================
    // 1. ПОРТЫ (Ports)
    // =========================================================================

    /// Входной порт навигационных битов.
    /// Ожидает поток, где значение > 0.5 означает логическую 1, иначе 0.
    gr::PortIn<T> nav_bits_in;

    /// Выходной порт комплексного IQ сигнала
    gr::PortOut<std::complex<T>> iq_out;

    /// Порт асинхронных сообщений для управления динамикой (доплер, фаза, многолучевость)
    gr::MsgInPort dynamics_in;

    // =========================================================================
    // 2. СВОЙСТВА (Properties)
    // =========================================================================

    GR_PROPERTY(int, prn) = 1;
    GR_PROPERTY(double, sample_rate) = 5e6;

    // =========================================================================
    // 3. КОНСТРУКТОР
    // =========================================================================

    GpsChannelBlock(int p = 1, double sr = 5e6)
        : prn(p),
          sample_rate(sr),
          code_gen_(p, sr, 0.0),
          carrier_nco_(0.0, sr, 0.0),
          current_nav_bit_{T(1.0)}
    {
        // Регистрация обработчика входящих сообщений
        this->register_handler("dynamics_in", [this](const gr::Message& msg) {
            this->handleDynamicsMessage(msg);
        });
    }

    // =========================================================================
    // 4. ОСНОВНОЙ МЕТОД ОБРАБОТКИ (Work)
    // =========================================================================

    work_return_code_t work(std::size_t requested_work) {
        std::ignore = requested_work;

        auto nav_bits_span = nav_bits_in.get_stream_values_available();
        auto iq_out_span   = iq_out.get_stream_buffer();

        // Определяем количество сэмплов для обработки (минимум из доступных)
        auto n_samples = std::min(nav_bits_span.size(), iq_out_span.size());
        if (n_samples == 0) {
            return work_return_code_t::INSUFFICIENT_INPUT_ITEMS;
        }

        // Оптимизация: переиспользуем внутренние буферы
        if (temp_baseband_.size() < n_samples) {
            temp_baseband_.resize(n_samples);
            temp_carrier_.resize(n_samples);
        }

        std::span<std::complex<T>> baseband_span(temp_baseband_.data(), n_samples);
        std::span<std::complex<T>> carrier_span(temp_carrier_.data(), n_samples);

        // ШАГ 1: Суммарный комплексный базовый сигнал (код + многолучевость)
        code_gen_.generateBaseband(baseband_span);

        // ШАГ 2: Комплексная несущая (NCO)
        carrier_nco_.generate(carrier_span);

        // ШАГ 3: Финальная модуляция: Базовый × Несущая × Навигационный бит
        T nav_bit = current_nav_bit_; // удержание (hold) последнего бита

        for (std::size_t i = 0; i < n_samples; ++i) {
            nav_bit = (nav_bits_span[i] > T(0.5)) ? T(1.0) : T(-1.0);
            current_nav_bit_ = nav_bit;
            iq_out_span[i] = baseband_span[i] * carrier_span[i] * nav_bit;
        }

        nav_bits_in.consume(n_samples);
        iq_out.publish(n_samples);

        return work_return_code_t::OK;
    }

    // =========================================================================
    // 5. УПРАВЛЕНИЕ ДИНАМИКОЙ (Message Handler)
    // =========================================================================

private:
    void handleDynamicsMessage(const gr::Message& msg) {
        const auto& data = msg.data;

        // Обновление доплеровского сдвига (в Гц)
        if (data.contains("doppler_hz")) {
            double doppler = std::get<double>(data.at("doppler_hz"));
            double sr = static_cast<double>(sample_rate);

            // Синхронно обновляем и код, и несущую для сохранения когерентности
            code_gen_.updateDoppler(doppler, sr);
            carrier_nco_.setFrequency(doppler, sr);
        }

        // Обновление абсолютной фазы кода (в чипах)
        if (data.contains("code_phase_chips")) {
            double phase = std::get<double>(data.at("code_phase_chips"));
            code_gen_.setPhase(phase);
        }

        // Динамическое добавление многолучевого компонента
        if (data.contains("add_multipath")) {
            const auto& mp_data = std::get<std::map<std::string, double>>(data.at("add_multipath"));
            gnss_sim::MultipathComponent<T> comp;
            comp.delay_chips = mp_data.at("delay_chips");
            comp.amplitude = mp_data.at("amplitude");
            comp.phase_offset_rad = mp_data.at("phase_offset_rad");

            code_gen_.addMultipathComponent(comp);
        }
    }

    // =========================================================================
    // 6. ВНУТРЕННЕЕ СОСТОЯНИЕ (State)
    // =========================================================================

    gnss_sim::MultipathCodeGenerator<gnss_sim::Constellation::GPS_L1_CA, T> code_gen_;
    gnss_sim::NCO<T> carrier_nco_;

    // Временные буферы для избежания аллокаций в методе work()
    std::vector<std::complex<T>> temp_baseband_;
    std::vector<std::complex<T>> temp_carrier_;

    // Состояние для удержания навигационного бита между вызовами work()
    T current_nav_bit_;
};

} // namespace gr::gnss_sim
