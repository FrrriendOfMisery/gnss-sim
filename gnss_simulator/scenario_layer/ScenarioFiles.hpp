#pragma once

#include "position.hpp"
#include "signal_config.hpp"
#include <string>
#include <vector>
#include <map>
#include <variant>
#include <optional>
#include <chrono>

namespace gnss_sim {

// ============================================================================
// Время
// ============================================================================
struct TimeConfig {
    std::string start_utc{"2026-11-22T12:00:00Z"};
    double duration_sec{60.0};
    double update_rate_hz{10.0};
};

// ============================================================================
// Траектории
// ============================================================================
struct StaticTrajectory {
    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m{0.0};
};

struct LinearTrajectory {
    double start_lat_deg{0.0}, start_lon_deg{0.0}, start_alt_m{0.0};
    double end_lat_deg{0.0}, end_lon_deg{0.0}, end_alt_m{0.0};
    double speed_mps{5.0};
};

struct CircularTrajectory {
    double center_lat_deg{0.0}, center_lon_deg{0.0}, center_alt_m{0.0};
    double radius_m{1000.0};
    double speed_mps{10.0};
    bool clockwise{true};
};

struct Waypoint {
    double time_sec{0.0};
    double lat_deg{0.0}, lon_deg{0.0}, alt_m{0.0};
};

struct WaypointsTrajectory {
    std::vector<Waypoint> waypoints;
    std::string interpolation{"linear"}; // linear | cubic | spline
};

struct NmeaTrajectory {
    std::string path;
    double time_offset_sec{0.0};
};

using TrajectoryVariant = std::variant<
    StaticTrajectory,
    LinearTrajectory,
    CircularTrajectory,
    WaypointsTrajectory,
    NmeaTrajectory
>;

// ============================================================================
// Часы и антенна
// ============================================================================
struct ClockConfig {
    double bias_s{0.0};
    double drift_sps{0.0};
};

struct AntennaConfig {
    double gain_dbi{0.0};
    std::string pattern{"isotropic"};
};

// ============================================================================
// Приёмник
// ============================================================================
struct ReceiverConfig {
    TrajectoryVariant trajectory{StaticTrajectory{}};
    ClockConfig clock;
    AntennaConfig antenna;
    double elevation_mask_deg{5.0};
};

// ============================================================================
// Эфемериды
// ============================================================================
struct BuiltinEphemeris {
    int num_satellites{24};
};

struct RinexNavEphemeris {
    std::string path;
    std::vector<int> prn_filter;
};

struct Sp3Ephemeris {
    std::string path;
};

using EphemerisVariant = std::variant<
    BuiltinEphemeris,
    RinexNavEphemeris,
    Sp3Ephemeris
>;

struct SignalPower {
    std::string name;
    bool enabled{true};
    double power_dbw{-130.0};
};

struct ConstellationConfig {
    std::string system;
    bool enabled{true};
    EphemerisVariant ephemeris{BuiltinEphemeris{}};
    std::vector<SignalPower> signals;
};

// ============================================================================
// Атмосфера
// ============================================================================
struct IonosphereConfig {
    std::string model{"klobuchar"}; // none | klobuchar | nequick
    std::vector<double> alpha;
    std::vector<double> beta;
};

struct TroposphereConfig {
    std::string model{"saastamoinen"};
    double temperature_c{15.0};
    double pressure_hpa{1013.25};
    double humidity_percent{50.0};
};

// ============================================================================
// Многолучевость
// ============================================================================
struct Reflector {
    double lat_deg{0.0}, lon_deg{0.0}, alt_m{0.0};
    std::string type{"vertical_wall"};
    double reflection_coefficient{0.5};
    double phase_shift_rad{0.0};
    double max_delay_chips{2.0};
};

struct MultipathConfig {
    bool enabled{false};
    std::string model{"none"};
    std::vector<Reflector> reflectors;
    std::string building_model;
    int max_reflections{3};
};

struct EnvironmentConfig {
    IonosphereConfig ionosphere;
    TroposphereConfig troposphere;
    MultipathConfig multipath;
};

// ============================================================================
// Помехи
// ============================================================================
struct JammerConfig {
    std::string type; // narrowband | swept | broadband
    double center_freq_hz{1575.42e6};
    double bandwidth_hz{1000.0};
    double power_dbw{-80.0};
    double start_time_sec{0.0};
    double duration_sec{30.0};
};

struct SpooferConfig {
    std::string type;
    int target_prn{0};
    double delay_offset_chips{0.0};
    double power_dbw{-125.0};
};

struct InterferenceConfig {
    bool enabled{false};
    std::vector<JammerConfig> jammers;
    std::vector<SpooferConfig> spoofers;
};

// ============================================================================
// Вывод
// ============================================================================
struct FileOutput {
    std::string path{"output.dat"};
    std::string format{"complex_float32"};
    bool overwrite{true};
};

struct HackrfOutput {
    double center_freq_hz{1575.42e6};
    double sample_rate_hz{5e6};
    double tx_gain_db{40.0};
    bool safe_mode{false};
    double bandwidth_hz{5e6};
};

struct LoggingConfig {
    bool enabled{true};
    std::string level{"INFO"};
    std::string file;
    bool log_pvt{false};
    bool log_ephemeris{false};
};

using OutputVariant = std::variant<
    std::monostate,  // none
    FileOutput,
    HackrfOutput
    // + UsrpOutput, LimesdrOutput, etc.
>;

struct OutputConfig {
    OutputVariant output;
    LoggingConfig logging;
};

// ============================================================================
// Симуляция
// ============================================================================
struct SimulationConfig {
    double sample_rate_hz{5e6};
    size_t max_channels{12};
    int num_threads{1};
    size_t buffer_size_samples{65536};
    uint64_t random_seed{0};
    double stats_interval_sec{1.0};
    std::optional<size_t> stop_after_samples;
};

// ============================================================================
// Метаданные
// ============================================================================
struct MetadataConfig {
    std::string version{"1.0"};
    std::string name;
    std::string author;
    std::string description;
};

// ============================================================================
// Главный сценарий
// ============================================================================
struct Scenario {
    MetadataConfig metadata;
    TimeConfig time;
    ReceiverConfig receiver;
    std::vector<ConstellationConfig> constellation;
    EnvironmentConfig environment;
    InterferenceConfig interference;
    OutputConfig output;
    SimulationConfig simulation;
};

} // namespace gnss_sim