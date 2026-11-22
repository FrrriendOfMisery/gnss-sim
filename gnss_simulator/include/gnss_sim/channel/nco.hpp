#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <span>

namespace gnss_sim {

/**
 * @brief Numerically Controlled Oscillator с compile-time таблицей синусов
 *
 * Целочисленный фазовый аккумулятор (без плавающей точки в критическом цикле).
 * @tparam T Тип данных сэмпла (float/double)
 */
template <typename T>
class NCO {
public:
    static constexpr std::size_t LUT_SIZE = 4096; // степень двойки

    /// phase_offset_rad — начальная фаза, sample_rate — Гц, freq_hz — стартовая частота
    NCO(double freq_hz = 0.0, double sample_rate = 5e6, double phase_offset_rad = 0.0) {
        buildLut();
        setFrequency(freq_hz, sample_rate);
        phase_acc_ = static_cast<std::uint32_t>(
            (phase_offset_rad / (2.0 * std::numbers::pi)) * PHASE_SCALE) % PHASE_SCALE;
    }

    /// Устанавливает частоту (Гц) относительно sample_rate (Гц)
    void setFrequency(double freq_hz, double sample_rate) {
        double cycles_per_sample = freq_hz / sample_rate;
        phase_step_ = static_cast<std::uint32_t>(
            std::llround(cycles_per_sample * static_cast<double>(PHASE_SCALE)));
    }

    /// Генерирует комплексную несущую exp(j*phase) в output
    void generate(std::span<std::complex<T>> output) {
        std::uint32_t phase = phase_acc_;
        const std::uint32_t step = phase_step_;
        constexpr std::uint32_t mask = LUT_SIZE - 1;

        for (auto& s : output) {
            // Индекс со линейной интерполяцией не обязателен — 4096 точек достаточно
            const std::uint32_t idx = (phase >> (32 - LUT_BITS)) & mask;
            s = std::complex<T>(static_cast<T>(cos_lut_[idx]),
                                static_cast<T>(sin_lut_[idx]));
            phase += step;
        }
        phase_acc_ = phase; // естественное переполнение uint32 == mod 2^32
    }

    /// Текущая накопленная фаза (радианы, mod 2pi)
    [[nodiscard]] double phase_rad() const {
        return static_cast<double>(phase_acc_) / PHASE_SCALE * 2.0 * std::numbers::pi;
    }

private:
    static constexpr std::size_t LUT_BITS = 12; // log2(4096)
    static constexpr double PHASE_SCALE = 4294967296.0; // 2^32

    std::array<double, LUT_SIZE> sin_lut_{};
    std::array<double, LUT_SIZE> cos_lut_{};

    std::uint32_t phase_acc_{0};
    std::uint32_t phase_step_{0};

    void buildLut() {
        for (std::size_t i = 0; i < LUT_SIZE; ++i) {
            double angle = static_cast<double>(i) / LUT_SIZE * 2.0 * std::numbers::pi;
            sin_lut_[i] = std::sin(angle);
            cos_lut_[i] = std::cos(angle);
        }
    }
};

} // namespace gnss_sim
