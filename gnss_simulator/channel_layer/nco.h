#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include "multipath_code_generator.hpp"
#include "nco.hpp"

namespace gr::gnss_sim {

template <typename T>
class GpsChannelBlock : public gr::Block<GpsChannelBlock<T>> {
private:
    // Композиция: блок содержит специализированные алгоритмы
    gnss_sim::MultipathCodeGenerator<gnss_sim::Constellation::GPS_L1_CA> code_gen_;
    gnss_sim::NCO<T> carrier_nco_;

public:
    gr::Port<T> nav_bits_in{this, "nav_bits", gr::Direction::IN};
    gr::Port<std::complex<T>> iq_out{this, "iq_out", gr::Direction::OUT};
    gr::MessagePort dynamics_in{this, "dynamics", gr::Direction::IN};

    gr::Property<int> prn{this, "prn", 1};
    gr::Property<double> sample_rate{this, "sample_rate", 5e6};

    GpsChannelBlock(int p = 1, double sr = 5e6) 
        : prn{this, "prn", p}, 
          sample_rate{this, "sample_rate", sr},
          code_gen_(p, sr, 0.0),
          carrier_nco_(0.0, sr, 0.0) // Начальная частота 0 Гц (базовая полоса)
    {
        dynamics_in.setHandler([this](const gr::Message& msg) {
            if (msg.data.contains("doppler_hz")) {
                double doppler = msg.data.at("doppler_hz").get<double>();
                // Обновляем и код, и несущую синхронно!
                code_gen_.updateDoppler(doppler, sample_rate);
                carrier_nco_.setFrequency(doppler, sample_rate);
            }
            if (msg.data.contains("code_phase")) {
                code_gen_.setPhase(msg.data.at("code_phase").get<double>());
            }
        });
    }

    gr::work_return_t work() {
        auto nav_bits = nav_bits_in.streamBuffer();
        auto iq_out_buf = iq_out.streamBuffer();
        
        auto n_samples = std::min(nav_bits.size(), iq_out_buf.size());
        if (n_samples == 0) return {0, 0};

        // 1. Генерируем комплексный код с учетом многолучевости (I и Q компоненты)
        std::vector<std::complex<T>> chips(n_samples);
        std::span<std::complex<T>> chip_span(chips);
        code_gen_.generate(chip_span);

        // 2. Генерируем несущую
        std::vector<std::complex<T>> carrier(n_samples);
        std::span<std::complex<T>> carrier_span(carrier);
        carrier_nco_.generate(carrier_span);

        // 3. Финальная модуляция: Навигационный бит * Код * Несущая
        for (size_t i = 0; i < n_samples; ++i) {
            T nav_symbol = (nav_bits[i] > 0.5) ? T(1.0) : T(-1.0);
            
            // Умножение комплексных чисел: (chip_i + j*chip_q) * (carr_i + j*carr_q) * nav
            iq_out_buf[i] = chip_span[i] * carrier_span[i] * nav_symbol;
        }

        return {n_samples, n_samples};
    }
};

} // namespace gr::gnss_sim