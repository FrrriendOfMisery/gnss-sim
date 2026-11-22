#pragma once

#include <cmath>
#include <numbers>

namespace gnss_sim {

/**
 * @brief 3D вектор в ECEF (Earth-Centered, Earth-Fixed) координатах
 * Также поддерживает геодезические координаты (WGS-84)
 */
struct Position {
    double x{0.0}; // ECEF X (метры)
    double y{0.0}; // ECEF Y (метры)
    double z{0.0}; // ECEF Z (метры)

    // WGS-84 константы
    static constexpr double WGS84_A = 6378137.0;           // Большая полуось (м)
    static constexpr double WGS84_F = 1.0 / 298.257223563; // Сжатие
    static constexpr double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Эксцентриситет^2

    Position() = default;
    Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    /**
     * @brief Создание из геодезических координат (WGS-84)
     * @param lat_rad Широта (радианы)
     * @param lon_rad Долгота (радианы)
     * @param alt_m   Высота над эллипсоидом (метры)
     */
    static Position fromGeodetic(double lat_rad, double lon_rad, double alt_m) {
        double sin_lat = std::sin(lat_rad);
        double cos_lat = std::cos(lat_rad);
        double sin_lon = std::sin(lon_rad);
        double cos_lon = std::cos(lon_rad);

        // Радиус кривизны в первом вертикале
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

        return Position{
            (N + alt_m) * cos_lat * cos_lon,
            (N + alt_m) * cos_lat * sin_lon,
            (N * (1.0 - WGS84_E2) + alt_m) * sin_lat
        };
    }

    /**
     * @brief Преобразование в геодезические координаты (итерационный метод Bowring)
     */
    void toGeodetic(double& lat_rad, double& lon_rad, double& alt_m) const {
        double p = std::sqrt(x * x + y * y);
        lon_rad = std::atan2(y, x);

        // Итерационное решение для широты
        double lat = std::atan2(z, p * (1.0 - WGS84_E2));
        for (int i = 0; i < 5; ++i) {
            double sin_lat = std::sin(lat);
            double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
            lat = std::atan2(z + WGS84_E2 * N * sin_lat, p);
        }
        lat_rad = lat;

        double sin_lat = std::sin(lat_rad);
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
        alt_m = p / std::cos(lat_rad) - N;
    }

    [[nodiscard]] double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Position operator-(const Position& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Position operator+(const Position& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Position operator*(double s) const {
        return {x * s, y * s, z * s};
    }

    [[nodiscard]] double dot(const Position& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

/**
 * @brief 3D вектор скорости (м/с) в ECEF
 */
struct Velocity {
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};

    Velocity() = default;
    Velocity(double vx_, double vy_, double vz_) : vx(vx_), vy(vy_), vz(vz_) {}

    Velocity operator-(const Velocity& other) const {
        return {vx - other.vx, vy - other.vy, vz - other.vz};
    }

    [[nodiscard]] double dot(const Position& p) const {
        return vx * p.x + vy * p.y + vz * p.z;
    }
};

} // namespace gnss_sim