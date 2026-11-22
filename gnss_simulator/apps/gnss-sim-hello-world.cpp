#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <fmt/core.h>

#include "gps_channel_block.hpp"
#include "nav_bit_source.hpp"
#include "scenario_manager.hpp"
#include "signal_combiner.hpp"
#include "gnss_sdr_file_sink.hpp"

int main() {
    using namespace gr;
    using namespace gr::gnss_sim;

    fmt::print("=== GNSS Simulator -> GNSS-SDR ===\n");
    fmt::print("Generating 60 seconds of GPS signal...\n\n");

    constexpr double SAMPLE_RATE = 5e6;      // 5 МГц (стандарт для GNSS-SDR)
    constexpr double DURATION_SEC = 60.0;    // 60 секунд
    constexpr size_t MAX_CHANNELS = 12;      // 12 каналов
    constexpr double UPDATE_RATE = 10.0;     // 10 Hz обновление геометрии

    Graph graph;

    // 1. ScenarioManager
    auto& scenario_mgr = graph.emplaceBlock<ScenarioManager<float>>(
        SAMPLE_RATE, UPDATE_RATE, MAX_CHANNELS
    );
    scenario_mgr.setReceiverPosition(55.75, 37.62, 150.0); // Москва

    // 2. Пул каналов
    std::vector<std::reference_wrapper<GpsChannelBlock<float>>> channels;
    std::vector<std::reference_wrapper<NavBitSource<float>>> nav_sources;

    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        auto& nav = graph.emplaceBlock<NavBitSource<float>>(SAMPLE_RATE);
        nav_sources.push_back(nav);

        auto& ch = graph.emplaceBlock<GpsChannelBlock<float>>(0, SAMPLE_RATE);
        channels.push_back(ch);

        graph.connect(nav.out, ch.nav_bits_in);
        graph.connect(scenario_mgr.channel_dynamics_out[i], ch.dynamics_in);
    }

    // 3. SignalCombiner (сумматор + шум)
    auto& combiner = graph.emplaceBlock<SignalCombiner<float>>(MAX_CHANNELS, -111.0);
    
    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        graph.connect(channels[i].get().iq_out, combiner.channel_inputs[i]);
    }

    // 4. FileSink для GNSS-SDR
    auto& file_sink = graph.emplaceBlock<GnssSdrFileSink<float>>(
        "gnss_sim_output.dat",
        SAMPLE_RATE,
        0.0  // Базовая полоса
    );
    graph.connect(combiner.combined_out, file_sink.data_in);

    // 5. Запуск
    fmt::print("Running simulation for {:.1f} seconds...\n", DURATION_SEC);
    fmt::print("Output file: gnss_sim_output.dat\n");
    fmt::print("Config file: gnss_sim_output.dat.conf\n\n");
    
    auto scheduler = scheduler::Simple(std::move(graph));
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < DURATION_SEC) {
        scheduler.run_once();
    }
    scheduler.stop();

    fmt::print("\n=== Simulation Complete ===\n");
    fmt::print("To run in GNSS-SDR:\n");
    fmt::print("  gnss-sdr --config_file=gnss_sim_output.dat.conf\n\n");

    return 0;
}