#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Property.hpp>

#include <complex>
#include <vector>
#include <random>
#include <span>
#include <numbers>

namespace gr::gnss_sim {

/**
 * @brief Сумматор сигналов от всех каналов + добавление AWGN шума
 * 
 * Принимает N комплексных входов от каналов, суммирует их,
 * добавляет белый гауссовский шум (опционально) и выдаёт один выход.
 */
template <typename T>
class SignalCombiner : public gr::Block<SignalCombiner<T>> {
public:
    // =========================================================================
    // ПОРТЫ
    // =========================================================================
    
    /// Вектор входных портов (по одному на каждый канал)
    std::vector<gr::Port<std::complex<T>>> channel_inputs;
    
    /// Выходной порт (суммарный сигнал)
    gr::Port<std::complex<T>> output{this, "output", gr::Direction::OUT};

    // =========================================================================
    // ПАРАМЕТРЫ
    // =========================================================================
    
    /// Количество каналов
    gr::Property<size_t> num_channels{this, "num_channels", 12};
    
    /// Мощность шума (C/N0 в дБ-Гц). 0 = без шума.
    /// Типичные значения: 30-50 дБ-Гц
    gr::Property<double> cn0_dbhz{this, "cn0_dbhz", 0.0};
    
    /// Частота дискретизации (Гц)
    gr::Property<double> sample_rate{this, "sample_rate", 5e6};

    // =========================================================================
    // КОНСТРУКТОР
    // =========================================================================
    
    SignalCombiner(size_t n_channels = 12, double cn0 = 0.0, double sr = 5e6)
        : num_channels{this, "num_channels", n_channels},
          cn0_dbhz{this, "cn0_dbhz", cn0},
          sample_rate{this, "sample_rate", sr}
    {
        // Создаём входные порты
        for (size_t i = 0; i < n_channels; ++i) {
            channel_inputs.emplace_back(this, "ch_in_" + std::to_string(i), gr::Direction::IN);
        }
        
        // Инициализация генератора случайных чисел
        rng_.seed(42); // Фиксированный seed для воспроизводимости
    }

    // =========================================================================
    // ОБРАБОТКА
    // =========================================================================
    
    gr::work_return_t work() {
        auto out_span = output.streamBuffer();
        auto n_samples = out_span.size();
        
        if (n_samples == 0) return {0, 0};

        // 1. Обнуляем выходной буфер
        for (size_t i = 0; i < n_samples; ++i) {
            out_span[i] = std::complex<T>(0.0, 0.0);
        }

        // 2. Суммируем все каналы
        size_t active_channels = 0;
        for (size_t ch = 0; ch < num_channels; ++ch) {
            auto in_span = channel_inputs[ch].streamBuffer();
            auto n_in = std::min(n_samples, in_span.size());
            
            for (size_t i = 0; i < n_in; ++i) {
                out_span[i] += in_span[i];
            }
            
            if (n_in > 0) active_channels++;
        }

        // 3. Добавление AWGN шума (если cn0_dbhz > 0)
        if (cn0_dbhz > 0.0) {
            addAWGN(out_span, n_samples);
        }

        // 4. Норм