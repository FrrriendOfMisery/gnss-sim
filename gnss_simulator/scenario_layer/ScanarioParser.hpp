#pragma once

#include "scenario_types.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>
#include <fmt/core.h>

namespace gnss_sim {

/**
 * @brief Парсер сценариев из YAML формата
 */
class ScenarioParser {
public:
    /**
     * @brief Загрузка сценария из файла
     */
    static Scenario loadFromFile(const std::string& path) {
        try {
            YAML::Node root = YAML::LoadFile(path);
            return parseScenario(root);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error(
                fmt::format("Failed to parse scenario file '{}': {}", path, e.what()));
        }
    }

    /**
     * @brief Загрузка сценария из строки
     */
    static Scenario loadFromString(const std::string& yaml_content) {
        try {
            YAML::Node root = YAML::Load(yaml_content);
            return parseScenario(root);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error(
                fmt::format("Failed to parse scenario: {}", e.what()));
        }
    }

    /**
     * @brief Валидация сценария
     */
    static std::vector<std::string> validate(const Scenario& scenario) {
        std::vector<std::string> errors;

        if (scenario.time.duration_sec <= 0) {
            errors.push_back("time.duration_sec must be positive");
        }
        if (scenario.simulation.sample_rate_hz <= 0) {
            errors.push_back("simulation.sample_rate_hz must be positive");
        }
        if (scenario.constellation.empty()) {
            errors.push_back("At least one constellation must be defined");
        }
        if (scenario.simulation.max_channels == 0) {
            errors.push_back("simulation.max_channels must be > 0");
        }

        return errors;
    }

private:
    static Scenario parseScenario(const YAML::Node& root) {
        Scenario scenario;

        if (root["metadata"]) scenario.metadata = parseMetadata(root["metadata"]);
        if (root["time"]) scenario.time = parseTime(root["time"]);
        if (root["receiver"]) scenario.receiver = parseReceiver(root["receiver"]);
        if (root["constellation"]) scenario.constellation = parseConstellation(root["constellation"]);
        if (root["environment"]) scenario.environment = parseEnvironment(root["environment"]);
        if (root["interference"]) scenario.interference = parseInterference(root["interference"]);
        if (root["output"]) scenario.output = parseOutput(root["output"]);
        if (root["simulation"]) scenario.simulation = parseSimulation(root["simulation"]);

        return scenario;
    }

    static MetadataConfig parseMetadata(const YAML::Node& node) {
        MetadataConfig cfg;
        if (node["version"]) cfg.version = node["version"].as<std::string>();
        if (node["name"]) cfg.name = node["name"].as<std::string>();
        if (node["author"]) cfg.author = node["author"].as<std::string>();
        if (node["description"]) cfg.description = node["description"].as<std::string>();
        return cfg;
    }

    static TimeConfig parseTime(const YAML::Node& node) {
        TimeConfig cfg;
        if (node["start"]) cfg.start_utc = node["start"].as<std::string>();
        if (node["duration_sec"]) cfg.duration_sec = node["duration_sec"].as<double>();
        if (node["update_rate_hz"]) cfg.update_rate_hz = node["update_rate_hz"].as<double>();
        return cfg;
    }

    static ReceiverConfig parseReceiver(const YAML::Node& node) {
        ReceiverConfig cfg;
        
        std::string traj_type = "static";
        if (node["trajectory"]) traj_type = node["trajectory"].as<std::string>();

        if (traj_type == "static") {
            StaticTrajectory t;
            if (node["position"]) {
                auto pos = node["position"].as<std::vector<double>>();
                if (pos.size() >= 3) {
                    t.lat_deg = pos[0];
                    t.lon_deg = pos[1];
                    t.alt_m = pos[2];
                }
            }
            cfg.trajectory = t;
        }
        else if (traj_type == "linear") {
            LinearTrajectory t;
            if (node["linear"]) {
                auto l = node["linear"];
                if (l["start"]) {
                    auto s = l["start"].as<std::vector<double>>();
                    t.start_lat_deg = s[0]; t.start_lon_deg = s[1]; t.start_alt_m = s[2];
                }
                if (l["end"]) {
                    auto e = l["end"].as<std::vector<double>>();
                    t.end_lat_deg = e[0]; t.end_lon_deg = e[1]; t.end_alt_m = e[2];
                }
                if (l["speed_mps"]) t.speed_mps = l["speed_mps"].as<double>();
            }
            cfg.trajectory = t;
        }
        else if (traj_type == "circular") {
            CircularTrajectory t;
            if (node["circular"]) {
                auto c = node["circular"];
                if (c["center"]) {
                    auto ctr = c["center"].as<std::vector<double>>();
                    t.center_lat_deg = ctr[0]; t.center_lon_deg = ctr[1]; t.center_alt_m = ctr[2];
                }
                if (c["radius_m"]) t.radius_m = c["radius_m"].as<double>();
                if (c["speed_mps"]) t.speed_mps = c["speed_mps"].as<double>();
                if (c["clockwise"]) t.clockwise = c["clockwise"].as<bool>();
            }
            cfg.trajectory = t;
        }
        else if (traj_type == "waypoints") {
            WaypointsTrajectory t;
            if (node["waypoints"]) {
                for (const auto& wp : node["waypoints"]) {
                    Waypoint w;
                    w.time_sec = wp["time_sec"].as<double>();
                    auto pos = wp["position"].as<std::vector<double>>();
                    w.lat_deg = pos[0]; w.lon_deg = pos[1]; w.alt_m = pos[2];
                    t.waypoints.push_back(w);
                }
                if (node["interpolation"]) t.interpolation = node["interpolation"].as<std::string>();
            }
            cfg.trajectory = t;
        }
        else if (traj_type == "nmea_file") {
            NmeaTrajectory t;
            if (node["nmea_file"]) {
                t.path = node["nmea_file"]["path"].as<std::string>();
                if (node["nmea_file"]["time_offset_sec"]) {
                    t.time_offset_sec = node["nmea_file"]["time_offset_sec"].as<double>();
                }
            }
            cfg.trajectory = t;
        }

        if (node["clock"]) {
            if (node["clock"]["bias_s"]) cfg.clock.bias_s = node["clock"]["bias_s"].as<double>();
            if (node["clock"]["drift_sps"]) cfg.clock.drift_sps = node["clock"]["drift_sps"].as<double>();
        }
        if (node["antenna"]) {
            if (node["antenna"]["gain_dbi"]) cfg.antenna.gain_dbi = node["antenna"]["gain_dbi"].as<double>();
            if (node["antenna"]["pattern"]) cfg.antenna.pattern = node["antenna"]["pattern"].as<std::string>();
        }
        if (node["elevation_mask_deg"]) cfg.elevation_mask_deg = node["elevation_mask_deg"].as<double>();

        return cfg;
    }

    static std::vector<ConstellationConfig> parseConstellation(const YAML::Node& node) {
        std::vector<ConstellationConfig> result;
        for (const auto& sys : node) {
            ConstellationConfig cfg;
            cfg.system = sys["system"].as<std::string>();
            if (sys["enabled"]) cfg.enabled = sys["enabled"].as<bool>();

            if (sys["ephemeris"]) {
                auto eph = sys["ephemeris"];
                std::string source = eph["source"].as<std::string>();
                
                if (source == "builtin") {
                    BuiltinEphemeris b;
                    if (eph["builtin"]["num_satellites"]) {
                        b.num_satellites = eph["builtin"]["num_satellites"].as<int>();
                    }
                    cfg.ephemeris = b;
                }
                else if (source == "rinex_nav") {
                    RinexNavEphemeris r;
                    r.path = eph["rinex_nav"]["path"].as<std::string>();
                    if (eph["rinex_nav"]["prn_filter"]) {
                        r.prn_filter = eph["rinex_nav"]["prn_filter"].as<std::vector<int>>();
                    }
                    cfg.ephemeris = r;
                }
                else if (source == "sp3") {
                    Sp3Ephemeris s;
                    s.path = eph["sp3"]["path"].as<std::string>();
                    cfg.ephemeris = s;
                }
            }

            if (sys["signals"]) {
                for (const auto& sig : sys["signals"]) {
                    SignalPower sp;
                    sp.name = sig["name"].as<std::string>();
                    if (sig["enabled"]) sp.enabled = sig["enabled"].as<bool>();
                    if (sig["power_dbw"]) sp.power_dbw = sig["power_dbw"].as<double>();
                    cfg.signals.push_back(sp);
                }
            }

            result.push_back(cfg);
        }
        return result;
    }

    static EnvironmentConfig parseEnvironment(const YAML::Node& node) {
        EnvironmentConfig cfg;
        
        if (node["ionosphere"]) {
            auto iono = node["ionosphere"];
            if (iono["model"]) cfg.ionosphere.model = iono["model"].as<std::string>();
            if (iono["custom"]) {
                if (iono["custom"]["alpha"]) cfg.ionosphere.alpha = iono["custom"]["alpha"].as<std::vector<double>>();
                if (iono["custom"]["beta"]) cfg.ionosphere.beta = iono["custom"]["beta"].as<std::vector<double>>();
            }
        }
        
        if (node["troposphere"]) {
            auto tropo = node["troposphere"];
            if (tropo["model"]) cfg.troposphere.model = tropo["model"].as<std::string>();
            if (tropo["weather"]) {
                auto w = tropo["weather"];
                if (w["temperature_c"]) cfg.troposphere.temperature_c = w["temperature_c"].as<double>();
                if (w["pressure_hpa"]) cfg.troposphere.pressure_hpa = w["pressure_hpa"].as<double>();
                if (w["humidity_percent"]) cfg.troposphere.humidity_percent = w["humidity_percent"].as<double>();
            }
        }
        
        if (node["multipath"]) {
            auto mp = node["multipath"];
            if (mp["enabled"]) cfg.multipath.enabled = mp["enabled"].as<bool>();
            if (mp["model"]) cfg.multipath.model = mp["model"].as<std::string>();
            
            if (mp["static_reflectors"]) {
                for (const auto& r : mp["static_reflectors"]) {
                    Reflector refl;
                    auto pos = r["position"].as<std::vector<double>>();
                    refl.lat_deg = pos[0]; refl.lon_deg = pos[1]; refl.alt_m = pos[2];
                    if (r["type"]) refl.type = r["type"].as<std::string>();
                    if (r["reflection_coefficient"]) refl.reflection_coefficient = r["reflection_coefficient"].as<double>();
                    if (r["phase_shift_rad"]) refl.phase_shift_rad = r["phase_shift_rad"].as<double>();
                    if (r["max_delay_chips"]) refl.max_delay_chips = r["max_delay_chips"].as<double>();
                    cfg.multipath.reflectors.push_back(refl);
                }
            }
            
            if (mp["urban_3d"]) {
                if (mp["urban_3d"]["building_model"]) {
                    cfg.multipath.building_model = mp["urban_3d"]["building_model"].as<std::string>();
                }
                if (mp["urban_3d"]["max_reflections"]) {
                    cfg.multipath.max_reflections = mp["urban_3d"]["max_reflections"].as<int>();
                }
            }
        }

        return cfg;
    }

    static InterferenceConfig parseInterference(const YAML::Node& node) {
        InterferenceConfig cfg;
        if (node["enabled"]) cfg.enabled = node["enabled"].as<bool>();
        
        if (node["jammers"]) {
            for (const auto& j : node["jammers"]) {
                JammerConfig jc;
                jc.type = j["type"].as<std::string>();
                if (j["center_freq_hz"]) jc.center_freq_hz = j["center_freq_hz"].as<double>();
                if (j["bandwidth_hz"]) jc.bandwidth_hz = j["bandwidth_hz"].as<double>();
                if (j["power_dbw"]) jc.power_dbw = j["power_dbw"].as<double>();
                if (j["start_time_sec"]) jc.start_time_sec = j["start_time_sec"].as<double>();
                if (j["duration_sec"]) jc.duration_sec = j["duration_sec"].as<double>();
                cfg.jammers.push_back(jc);
            }
        }
        
        if (node["spoofers"]) {
            for (const auto& s : node["spoofers"]) {
                SpooferConfig sc;
                sc.type = s["type"].as<std::string>();
                if (s["target_prn"]) sc.target_prn = s["target_prn"].as<int>();
                if (s["delay_offset_chips"]) sc.delay_offset_chips = s["delay_offset_chips"].as<double>();
                if (s["power_dbw"]) sc.power_dbw = s["power_dbw"].as<double>();
                cfg.spoofers.push_back(sc);
            }
        }

        return cfg;
    }

    static OutputConfig parseOutput(const YAML::Node& node) {
        OutputConfig cfg;
        
        std::string type = "none";
        if (node["type"]) type = node["type"].as<std::string>();
        
        if (type == "file" && node["file"]) {
            FileOutput fo;
            if (node["file"]["path"]) fo.path = node["file"]["path"].as<std::string>();
            if (node["file"]["format"]) fo.format = node["file"]["format"].as<std::string>();
            if (node["file"]["overwrite"]) fo.overwrite = node["file"]["overwrite"].as<bool>();
            cfg.output = fo;
        }
        else if (type == "hackrf" && node["hackrf"]) {
            HackrfOutput ho;
            if (node["hackrf"]["center_freq_hz"]) ho.center_freq_hz = node["hackrf"]["center_freq_hz"].as<double>();
            if (node["hackrf"]["sample_rate_hz"]) ho.sample_rate_hz = node["hackrf"]["sample_rate_hz"].as<double>();
            if (node["hackrf"]["tx_gain_db"]) ho.tx_gain_db = node["hackrf"]["tx_gain_db"].as<double>();
            if (node["hackrf"]["safe_mode"]) ho.safe_mode = node["hackrf"]["safe_mode"].as<bool>();
            if (node["hackrf"]["bandwidth_hz"]) ho.bandwidth_hz = node["hackrf"]["bandwidth_hz"].as<double>();
            cfg.output = ho;
        }

        if (node["logging"]) {
            auto l = node["logging"];
            if (l["enabled"]) cfg.logging.enabled = l["enabled"].as<bool>();
            if (l["level"]) cfg.logging.level = l["level"].as<std::string>();
            if (l["file"]) cfg.logging.file = l["file"].as<std::string>();
            if (l["log_pvt"]) cfg.logging.log_pvt = l["log_pvt"].as<bool>();
            if (l["log_ephemeris"]) cfg.logging.log_ephemeris = l["log_ephemeris"].as<bool>();
        }

        return cfg;
    }

    static SimulationConfig parseSimulation(const YAML::Node& node) {
        SimulationConfig cfg;
        if (node["sample_rate_hz"]) cfg.sample_rate_hz = node["sample_rate_hz"].as<double>();
        if (node["max_channels"]) cfg.max_channels = node["max_channels"].as<size_t>();
        if (node["num_threads"]) cfg.num_threads = node["num_threads"].as<int>();
        if (node["buffer_size_samples"]) cfg.buffer_size_samples = node["buffer_size_samples"].as<size_t>();
        if (node["random_seed"]) cfg.random_seed = node["random_seed"].as<uint64_t>();
        if (node["stats_interval_sec"]) cfg.stats_interval_sec = node["stats_interval_sec"].as<double>();
        if (node["stop_after_samples"]) cfg.stop_after_samples = node["stop_after_samples"].as<size_t>();
        return cfg;
    }
};

} // namespace gnss_sim