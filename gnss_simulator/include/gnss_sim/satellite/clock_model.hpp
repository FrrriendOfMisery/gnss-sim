#pragma once

#include "../core/common.hpp"

namespace gnss_sim {

/**
 * @brief Абстрактная модель часов спутника (стратегия)
 */
class ClockModel {
public:
    virtual ~ClockModel() = default;

    /// Смещение часов (секунды) в момент time_sec
    virtual double getBias(double time_sec) const = 0;

    /// Дрейф часов (s/s)
    virtual double getDrift(double time_sec) const = 0;

    virtual std::unique_ptr<ClockModel> clone() const = 0;
};

/**
 * @brief Идеальные часы (нулевые смещение и дрейф)
 */
class IdealClock : public ClockModel {
public:
    double getBias(double /*time_sec*/) const override { return 0.0; }
    double getDrift(double /*time_sec*/) const override { return 0.0; }
    std::unique_ptr<ClockModel> clone() const override {
        return std::make_unique<IdealClock>(*this);
    }
};

/**
 * @brief Простая модель с фиксированными bias/drift (для тестов)
 */
class ConstantClock : public ClockModel {
public:
    ConstantClock(double bias_s = 0.0, double drift_sps = 0.0)
        : bias_s_(bias_s), drift_sps_(drift_sps) {}

    double getBias(double time_sec) const override {
        return bias_s_ + drift_sps_ * time_sec;
    }
    double getDrift(double /*time_sec*/) const override { return drift_sps_; }

    std::unique_ptr<ClockModel> clone() const override {
        return std::make_unique<ConstantClock>(*this);
    }

private:
    double bias_s_;
    double drift_sps_;
};

} // namespace gnss_sim
