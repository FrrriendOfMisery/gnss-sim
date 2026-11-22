#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Property.hpp>

#include <complex>
#include <span>
#include <vector>
#include <numbers>

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
    /// Если частота этого порта ниже sample_rate, блок удерживает последнее значение.
    gr::Port<T> nav_bits_in{this, "nav_bits", gr::Direction::IN};
    
    /// Выходной порт комплексного IQ сигнала
    gr::Port<std::complex<T>> iq_out{this, "iq_out", gr::Direction::OUT};
    
    /// Порт асинхронных сообщений для управления динамикой (доплер, фаза, многолучевость)
    gr::MessagePort dynamics_in{this, "dynamics", gr::Direction::IN};

    // =========================================================================
    // 2. СВОЙСТВА (Properties) - управляются извне (например, из Python или GUI)
    // =========================================================================
    
    gr::Property<int> prn{this, "prn", 1};
    gr::Property<double> sample_rate{this, "sample_rate", 5e6};

    // =========================================================================
    // 3. КОНСТРУКТОР
    // =========================================================================
    
    GpsChannelBlock(int p = 1, double sr = 5e6)
        : prn{this, "prn", p},
          sample_rate{this, "sample_rate", sr},
          code_gen_(p, sr, 0.0),
          carrier_nco_(0.0, sr, 0.0),
          current_nav_bit_{T(1.0)}
    {
        // Регистрация обработчика входящих сообщений
        dynamics_in.setHandler([this](const gr::Message& msg) {
            this->handleDynamicsMessage(msg);
        });
    }

    // =========================================================================
    // 4. ОСНОВНОЙ МЕТОД ОБРАБОТКИ (Work)
    // =========================================================================
    
    gr::work_return_t work() {
        // Получаем zero-copy доступ к буферам портов
        auto nav_bits_span = nav_bits_in.streamBuffer();
        auto iq_out_span = iq_out.streamBuffer();
        
        // Определяем количество сэмплов для обработки (минимум из доступных)
        auto n_samples = std::min(nav_bits_span.size(), iq_out_span.size());
        if (n_samples == 0) {
            return {0, 0}; // Нечего обрабатывать
        }

        // Оптимизация: переиспользуем внутренние буферы, чтобы избежать аллокаций в цикле work()
        if (temp_baseband_.size() < n_samples) {
            temp_baseband_.resize(n_samples);
            temp_carrier_.resize(n_samples);
        }

        std::span<std::complex<T>> baseband_span(temp_baseband_.data(), n_samples);
        std::span<std::complex<T>> carrier_span(temp_carrier_.data(), n_samples);

        // ШАГ 1: Генерируем суммарный комплексный базовый сигнал 
        // (Код каждого пути умножен на свою комплексную амплитуду и сложен)
        code_gen_.generateBaseband(baseband_span);

        // ШАГ 2: Генерируем комплексную несущую (NCO)
        carrier_nco_.generate(carrier_span);

        // ШАГ 3: Финальная модуляция: Базовый сигнал × Несущая × Навигационный бит
        T nav_bit = current_nav_bit_; // Используем удержание (hold) последнего бита
        
        for (size_t i = 0; i < n_samples; ++i) {
            // Обновляем текущий бит, если во входном потоке есть новые данные
            if (i < nav_bits_span.size()) {
                nav_bit = (nav_bits_span[i] > T(0.5)) ? T(1.0) : T(-1.0);
                current_nav_bit_ = nav_bit;
            }
            
            // Одно комплексное умножение на сэмпл
            // baseband_span[i] уже содержит сумму всех путей с их амплитудами и фазами
            iq_out_span[i] = baseband_span[i] * carrier_span[i] * nav_bit;
        }

        // Сообщаем планировщику GR4, сколько сэмплов мы потребили и произвели
        return {n_samples, n_samples};
    }

    // =========================================================================
    // 5. УПРАВЛЕНИЕ ДИНАМИКОЙ (Message Handler)
    // =========================================================================
    
private:
    void handleDynamicsMessage(const gr::Message& msg) {
        // Обновление доплеровского сдвига (в Гц)
        if (msg.data.contains("doppler_hz")) {
            double doppler = msg.data.at("doppler_hz").get<double>();
            double sr = sample_rate;
            
            // Синхронно обновляем и код, и несущую для сохранения когерентности
            code_gen_.updateDoppler(doppler, sr);
            carrier_nco_.setFrequency(doppler, sr);
        }
        
        // Обновление абсолютной фазы кода (в чипах)
        if (msg.data.contains("code_phase_chips")) {
            double phase = msg.data.at("code_phase_chips").get<double>();
            code_gen_.setPhase(phase);
        }

        // Пример: Добавление многолучевого компонента динамически
        if (msg.data.contains("add_multipath")) {
            const auto& mp_data = msg.data.at("add_multipath").get<std::map<std::string, double>>();
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

// =========================================================================
// 7. РЕГИСТРАЦИЯ БЛОКА В GNU RADIO 4
// =========================================================================
// Этот макрос позволяет GR4 создавать экземпляры блока через фабрику
// и использовать его в Python или GRC (GNU Radio Companion)
GR_REGISTER_BLOCK(gr::gnss_sim::GpsChannelBlock, (float, double))

} // namespace gr::gnss_sim