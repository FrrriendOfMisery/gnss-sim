#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Property.hpp>

#include <cstdint>
#include <vector>

namespace gr::gnss_sim {

/**
 * @brief Источник навигационных битов GPS L1 C/A (50 бит/с)
 *
 * Формирует поток псевдослучайных (детерминированных) навигационных символов
 * {+1, -1} с частотой 50 Гц, апсемплированных до sample_rate.
 */
template <typename T>
class NavBitSource : public gr::Block<NavBitSource<T>> {
public:
    gr::PortOut<T> nav_bits_out;

    GR_PROPERTY(double, sample_rate) = 5e6;
    GR_PROPERTY(int, prn) = 1;

    NavBitSource(double sr = 5e6, int p = 1)
        : sample_rate(sr), prn(p) {
        rebuildBits();
    }

    work_return_code_t work(std::size_t requested_work) {
        std::ignore = requested_work;
        auto out = nav_bits_out.get_stream_buffer();
        if (out.empty()) {
            return work_return_code_t::DONE; // sink-like source backpressure
        }

        const std::size_t n = out.size();
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = bits_[bit_index_];
            samples_in_bit_++;
            if (samples_in_bit_ >= samples_per_bit_) {
                samples_in_bit_ = 0;
                bit_index_ = (bit_index_ + 1) % bits_.size();
            }
        }
        nav_bits_out.publish(n);
        return work_return_code_t::OK;
    }

private:
    std::vector<T> bits_;
    std::size_t bit_index_{0};
    std::size_t samples_in_bit_{0};
    std::size_t samples_per_bit_{1};

    static constexpr double NAV_RATE_BPS = 50.0;

    void rebuildBits() {
        double sr = static_cast<double>(sample_rate);
        samples_per_bit_ = static_cast<std::size_t>(sr / NAV_RATE_BPS);
        if (samples_per_bit_ == 0) samples_per_bit_ = 1;

        // Детерминированная последовательность из 1500 бит (30 секунд данных)
        std::uint32_t state = 0x12345678u ^ (static_cast<std::uint32_t>(prn) * 0x9E3779B1u);
        bits_.resize(1500);
        for (auto& b : bits_) {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            b = (state & 1u) ? T(1.0) : T(-1.0);
        }
    }
};

} // namespace gr::gnss_sim
