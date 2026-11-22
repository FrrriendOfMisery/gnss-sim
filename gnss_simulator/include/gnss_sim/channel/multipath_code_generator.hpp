#pragma once

#include "../core/common.hpp"
#include "../core/constellation_traits.hpp"

namespace gnss_sim {

// ============================================================================
// 1. Компонент многолучевости с предвычисленной комплексной амплитудой
// ============================================================================
template <typename T>
struct MultipathComponent
{
    double delay_chips{0.0};       // Задержка в чипах
    double amplitude{1.0};         // Амплитуда (0.0 - 1.0)
    double phase_offset_rad{0.0};  // Фазовый сдвиг (радианы)

    // Предвычисленная комплексная амплитуда: A * e^(j*phi)
    std::complex<T> complex_amp{1.0, 0.0};

    void updateComplexAmp()
    {
        complex_amp = std::complex<T>(static_cast<T>(amplitude * std::cos(phase_offset_rad)), static_cast<T>(amplitude * std::sin(phase_offset_rad)));
    }
};

// ============================================================================
// 2. Генератор кода с многолучевостью (Оптимизированный)
// ============================================================================
template <Constellation C, typename T = float>
class MultipathCodeGenerator
{
private:
    static constexpr int CODE_LEN = ConstellationTraits<C>::CODE_LENGTH;
    static constexpr int FRACTIONAL_BITS = 16;
    static constexpr uint32_t PHASE_SCALE = 1U << FRACTIONAL_BITS;
    static constexpr uint32_t MAX_PHASE = CODE_LEN * PHASE_SCALE;

    std::array<int8_t, CODE_LEN> code_lut_;

    // Прямой путь (LOS)
    MultipathComponent<T> los_component_;
    uint32_t los_phase_acc_;

    // Отраженные пути
    std::vector<MultipathComponent<T>> mp_components_;
    std::vector<uint32_t> mp_phase_accs_;

    uint32_t phase_step_;

public:
    MultipathCodeGenerator(int prn, double sample_rate, double initial_phase_chips = 0.0)
        : los_phase_acc_(0), phase_step_(0)
    {
        ConstellationTraits<C>::generateLUT(prn, code_lut_);

        los_component_.amplitude = 1.0;
        los_component_.delay_chips = 0.0;
        los_component_.phase_offset_rad = 0.0;
        los_component_.updateComplexAmp();

        setPhase(initial_phase_chips);
        updateDoppler(0.0, sample_rate);
    }

    void setPhase(double chips) {
        if (chips < 0) chips = std::fmod(chips, CODE_LEN) + CODE_LEN;
        uint32_t int_chips = static_cast<uint32_t>(std::floor(chips)) % CODE_LEN;
        uint32_t frac_bits = static_cast<uint32_t>(std::round((chips - std::floor(chips)) * PHASE_SCALE));
        los_phase_acc_ = (int_chips << FRACTIONAL_BITS) | frac_bits;

        // Синхронизируем фазы отражений
        for (size_t i = 0; i < mp_components_.size(); ++i) {
            double mp_phase = chips - mp_components_[i].delay_chips;
            if (mp_phase < 0) mp_phase = std::fmod(mp_phase, CODE_LEN) + CODE_LEN;
            uint32_t mp_int = static_cast<uint32_t>(std::floor(mp_phase)) % CODE_LEN;
            uint32_t mp_frac = static_cast<uint32_t>(std::round((mp_phase - std::floor(mp_phase)) * PHASE_SCALE));
            mp_phase_accs_[i] = (mp_int << FRACTIONAL_BITS) | mp_frac;
        }
    }

    void updateDoppler(double doppler_hz, double sample_rate) {
        constexpr double CARRIER_FREQ = ConstellationTraits<C>::CARRIER_FREQ_HZ;
        constexpr double NOMINAL_CODE_RATE = ConstellationTraits<C>::CHIPPING_RATE_HZ;

        double actual_code_rate = NOMINAL_CODE_RATE * (1.0 + doppler_hz / CARRIER_FREQ);
        phase_step_ = static_cast<uint32_t>(std::round((actual_code_rate / sample_rate) * PHASE_SCALE));
    }

    size_t addMultipathComponent(const MultipathComponent<T>& component) {
        MultipathComponent<T> comp = component;
        comp.updateComplexAmp();
        mp_components_.push_back(comp);

        double mp_phase = getCurrentPhaseChips() - comp.delay_chips;
        if (mp_phase < 0) mp_phase = std::fmod(mp_phase, CODE_LEN) + CODE_LEN;
        uint32_t mp_int = static_cast<uint32_t>(std::floor(mp_phase)) % CODE_LEN;
        uint32_t mp_frac = static_cast<uint32_t>(std::round((mp_phase - std::floor(mp_phase)) * PHASE_SCALE));
        mp_phase_accs_.push_back((mp_int << FRACTIONAL_BITS) | mp_frac);

        return mp_components_.size() - 1;
    }

    /**
     * @brief Генерирует СУММАРНЫЙ комплексный базовый сигнал:
     * Sum( Real_Code_i * Complex_Amp_i )
     *
     * Этот результат затем нужно умножить только на NCO в блоке GR4.
     */
    void generateBaseband(std::span<std::complex<T>> output) {
        const int8_t* lut_ptr = code_lut_.data();

        uint32_t los_phase = los_phase_acc_;
        const uint32_t step = phase_step_;
        const uint32_t max_p = MAX_PHASE;

        // Локальные копии для векторизации
        std::vector<uint32_t> mp_phases = mp_phase_accs_;
        const size_t mp_count = mp_components_.size();

        for (size_t i = 0; i < output.size(); ++i) {
            // 1. Начинаем с прямого пути (LOS)
            uint32_t los_idx = los_phase >> FRACTIONAL_BITS;
            T chip_los = static_cast<T>(lut_ptr[los_idx]);

            std::complex<T> sum = chip_los * los_component_.complex_amp;

            // 2. Добавляем отраженные пути
            for (size_t j = 0; j < mp_count; ++j) {
                uint32_t mp_idx = mp_phases[j] >> FRACTIONAL_BITS;
                T chip_mp = static_cast<T>(lut_ptr[mp_idx]);

                sum += chip_mp * mp_components_[j].complex_amp;

                mp_phases[j] += step;
                if (mp_phases[j] >= max_p) mp_phases[j] -= max_p;
            }

            output[i] = sum;

            // 3. Инкремент фазы LOS
            los_phase += step;
            if (los_phase >= max_p) los_phase -= max_p;
        }

        los_phase_acc_ = los_phase;
        mp_phase_accs_ = mp_phases;
    }

    [[nodiscard]] double getCurrentPhaseChips() const {
        double int_part = los_phase_acc_ >> FRACTIONAL_BITS;
        double frac_part = (los_phase_acc_ & (PHASE_SCALE - 1)) / static_cast<double>(PHASE_SCALE);
        return int_part + frac_part;
    }
};

} // namespace gnss_sim
