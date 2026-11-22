#pragma once

#include "common.hpp"

namespace gnss_sim {

/**
 * @brief Поддерживаемые ГНСС / сигналы
 */
enum class Constellation {
    GPS_L1_CA,
    GPS_L2C,
    GPS_L5,
    GAL_E1_OS,
    GAL_E5A,
    GLO_L1_OF,
    BDS_B1I
};

/**
 * @brief Compile-time характеристики сигнала + генерация LUT кода
 *
 * Специализации должны предоставлять:
 *  - static constexpr int    CODE_LENGTH      (чипов)
 *  - static constexpr double CHIPPING_RATE_HZ (Гц)
 *  - static constexpr double CARRIER_FREQ_HZ  (Гц)
 *  - static void generateLUT(int prn, std::array<int8_t, CODE_LENGTH>& out)
 */
template <Constellation C>
struct ConstellationTraits;

// ----------------------------------------------------------------------------
// GPS L1 C/A: 1023 чипа, 1.023 МГц, 1575.42 МГц (Gold code G1^G2)
// ----------------------------------------------------------------------------
template <>
struct ConstellationTraits<Constellation::GPS_L1_CA> {
    static constexpr int    CODE_LENGTH      = 1023;
    static constexpr double CHIPPING_RATE_HZ = 1.023e6;
    static constexpr double CARRIER_FREQ_HZ  = 1575.42e6;

    /// Генерирует реальный Gold-код C/A для PRN 1..32
    static void generateLUT(int prn, std::array<int8_t, CODE_LENGTH>& out) {
        if (prn < 1 || prn > 32) prn = 1;

        // Регистры сдвига G1 (10 бит) и G2 (10 бит), единицы при старте
        std::uint32_t g1 = 0x3FFu;
        std::uint32_t g2 = 0x3FFu;

        // Полиномы обратных связей (маски отводов):
        // G1: x^9 + x^5 + 1 -> отводы 10,3 (0-based bit positions 9 и 2... )
        constexpr std::uint32_t G1_TAPS = (1u << 9) | (1u << 2);   // bits 10 and 3
        constexpr std::uint32_t G2_TAPS = (1u << 9) | (1u << 8) | (1u << 6) |
                                          (1u << 5) | (1u << 2) | 1u; // 10,9,7,6,3,1

        // Отводы сумматора G2 для конкретного PRN (ICD-GPS-200, Table)
        // Таблица пар отводов (1-based) для PRN 1..32:
        static constexpr int TAP_TABLE[32][2] = {
            {2, 6}, {3, 7}, {4, 8}, {5, 9}, {6, 10}, {1, 8}, {2, 9}, {3, 10},
            {4, 1}, {5, 2}, {6, 3}, {7, 4}, {8, 5}, {9, 6}, {10, 7}, {1, 9},
            {2, 10}, {3, 1}, {4, 2}, {5, 3}, {6, 4}, {7, 5}, {8, 6}, {9, 7},
            {10, 8}, {1, 6}, {2, 7}, {3, 8}, {4, 9}, {5, 10}, {6, 1}, {7, 2}
        };
        const int t1 = TAP_TABLE[prn - 1][0];
        const int t2 = TAP_TABLE[prn - 1][1];

        for (int i = 0; i < CODE_LENGTH; ++i) {
            // Выходные чипы ДО сдвига (bit index tap-1, т.к. бит 0 = выход регистра)
            std::uint32_t c1 = (g1 >> 9) & 1u; // выход G1 = Q10
            std::uint32_t g2_out = ((g2 >> (t1 - 1)) ^ (g2 >> (t2 - 1))) & 1u;

            out[i] = (c1 ^ g2_out) ? int8_t(-1) : int8_t(1); // инверсия: 0->+1, 1->-1

            // Сдвиг с обратной связью
            std::uint32_t fb1 = static_cast<std::uint32_t>(__builtin_parity(g1 & G1_TAPS));
            std::uint32_t fb2 = (__builtin_parity(g2 & G2_TAPS));
            g1 = (g1 >> 1) | (fb1 << 9);
            g2 = (g2 >> 1) | (fb2 << 9);
        }
    }
};

// ----------------------------------------------------------------------------
// Galileo E1 OS: 4092 чипа, 1.023 МГц (упрощённый детерминированный код)
// ----------------------------------------------------------------------------
template <>
struct ConstellationTraits<Constellation::GAL_E1_OS> {
    static constexpr int    CODE_LENGTH      = 4092;
    static constexpr double CHIPPING_RATE_HZ = 1.023e6;
    static constexpr double CARRIER_FREQ_HZ  = 1575.42e6;

    static void generateLUT(int prn, std::array<int8_t, CODE_LENGTH>& out) {
        // Упрощение: детерминированный псевдослучайный код (не официальный VM code)
        std::uint32_t state = 0x9E3779B9u ^ (static_cast<std::uint32_t>(prn) * 2654435761u);
        for (int i = 0; i < CODE_LENGTH; ++i) {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            out[i] = (state & 1u) ? int8_t(-1) : int8_t(1);
        }
    }
};

// ----------------------------------------------------------------------------
// GLONASS L1 OF: 511 чипов, 0.511 МГц, базовая 1602 МГц (FDMA)
// ----------------------------------------------------------------------------
template <>
struct ConstellationTraits<Constellation::GLO_L1_OF> {
    static constexpr int    CODE_LENGTH      = 511;
    static constexpr double CHIPPING_RATE_HZ = 0.511e6;
    static constexpr double CARRIER_FREQ_HZ  = 1602e6;

    static void generateLUT(int prn, std::array<int8_t, CODE_LENGTH>& out) {
        std::uint32_t state = 0x85EBCA6Bu ^ (static_cast<std::uint32_t>(prn) * 2246822519u);
        for (int i = 0; i < CODE_LENGTH; ++i) {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            out[i] = (state & 1u) ? int8_t(-1) : int8_t(1);
        }
    }
};

} // namespace gnss_sim
