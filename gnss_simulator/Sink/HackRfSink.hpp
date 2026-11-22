#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Property.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Version.hpp>

#include <complex>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <fmt/core.h>

namespace gr::gnss_sim {

/**
 * @brief Sink для HackRF (и других SDR) через SoapySDR
 * 
 * Поддерживает:
 * - HackRF One (макс. sample rate: 20 МГц)
 * - USRP (через SoapyUHD)
 * - LimeSDR (через SoapyLime)
 * - Любой другой SDR с SoapySDR драйвером
 * 
 * @tparam T Тип входных данных (обычно int8_t для HackRF)
 */
template <typename T>
class HackRFSink : public gr::Block<HackRFSink<T>> {
public:
    gr::Port<std::complex<T>> data_in{this, "data_in", gr::Direction::IN};

    gr::Property<double> center_freq{this, "center_freq", 1575.42e6};
    gr::Property<double> sample_rate{this, "sample_rate", 5e6};
    gr::Property<double> tx_gain{this, "tx_gain", 40.0};  // VGA gain (0-47 dB для HackRF)
    gr::Property<double> bandwidth{this, "bandwidth", 5e6};
    gr::Property<std::string> device_args{this, "device_args", "driver=hackrf"};

    HackRFSink(double center_freq_hz = 1575.42e6, 
               double sample_rate_hz = 5e6,
               double gain_db = 40.0,
               const std::string& args = "driver=hackrf")
        : center_freq{this, "center_freq", center_freq_hz},
          sample_rate{this, "sample_rate", sample_rate_hz},
          tx_gain{this, "tx_gain", gain_db},
          bandwidth{this, "bandwidth", sample_rate_hz},
          device_args{this, "device_args", args}
    {}

    ~HackRFSink() {
        stop();
    }

    /**
     * @brief Инициализация SDR устройства
     */
    bool start() {
        if (device_ != nullptr) return true;

        try {
            fmt::print("[HackRFSink] Loading SoapySDR modules...\n");
            SoapySDR::loadModules();

            fmt::print("[HackRFSink] Finding devices with args: {}\n", 
                       static_cast<std::string>(device_args));
            
            auto results = SoapySDR::Device::enumerate(static_cast<std::string>(device_args));
            if (results.empty()) {
                fmt::print(stderr, "[HackRFSink] ERROR: No SDR devices found!\n");
                return false;
            }

            fmt::print("[HackRFSink] Found {} device(s):\n", results.size());
            for (const auto& r : results) {
                fmt::print("  - {}\n", r.toString());
            }

            // Открываем устройство
            device_ = SoapySDR::Device::make(results[0]);
            
            // Проверяем возможности
            auto directions = device_->getStreamDirections(SOAPY_SDR_TX, 0);
            if (!(directions & SOAPY_SDR_TX)) {
                fmt::print(stderr, "[HackRFSink] ERROR: Device does not support TX!\n");
                stop();
                return false;
            }

            // Применяем настройки
            applySettings();

            // Настраиваем поток
            std::string stream_format = getStreamFormat();
            fmt::print("[HackRFSink] Setting up TX stream (format: {}, channel: 0)\n", stream_format);
            
            stream_ = device_->setupStream(SOAPY_SDR_TX, stream_format, {0});
            if (stream_ == nullptr) {
                fmt::print(stderr, "[HackRFSink] ERROR: Failed to setup stream!\n");
                stop();
                return false;
            }

            // Активируем поток
            int ret = device_->activateStream(stream_);
            if (ret != 0) {
                fmt::print(stderr, "[HackRFSink] ERROR: Failed to activate stream: {}\n", 
                           SoapySDR::errToStr(ret));
                stop();
                return false;
            }

            fmt::print("[HackRFSink] Device initialized successfully!\n");
            fmt::print("  Center freq: {:.3f} MHz\n", static_cast<double>(center_freq) / 1e6);
            fmt::print("  Sample rate: {:.3f} MHz\n", static_cast<double>(sample_rate) / 1e6);
            fmt::print("  TX gain:     {:.1f} dB\n", static_cast<double>(tx_gain));
            fmt::print("  Bandwidth:   {:.3f} MHz\n", static_cast<double>(bandwidth) / 1e6);

            is_running_ = true;
            return true;

        } catch (const std::exception& e) {
            fmt::print(stderr, "[HackRFSink] ERROR: {}\n", e.what());
            stop();
            return false;
        }
    }

    /**
     * @brief Остановка и освобождение устройства
     */
    void stop() {
        if (stream_ != nullptr && device_ != nullptr) {
            device_->deactivateStream(stream_);
            device_->closeStream(stream_);
            stream_ = nullptr;
        }
        if (device_ != nullptr) {
            SoapySDR::Device::unmake(device_);
            device_ = nullptr;
        }
        is_running_ = false;
    }

    gr::work_return_t work() {
        // Инициализация при первом вызове
        if (!is_running_) {
            if (!start()) {
                fmt::print(stderr, "[HackRFSink] Failed to start, stopping block\n");
                return gr::Error("Failed to initialize SDR device");
            }
        }

        auto in_span = data_in.streamBuffer();
        auto n_samples = in_span.size();
        if (n_samples == 0) return {0, 0};

        // Ограничиваем размер пакета (HackRF работает лучше с буферами по 65536)
        constexpr size_t MAX_PACKET_SIZE = 65536;
        size_t samples_to_write = std::min(n_samples, MAX_PACKET_SIZE);

        // Передаём данные в SDR
        void* buffs[] = {const_cast<void*>(static_cast<const void*>(in_span.data()))};
        int flags = 0;
        long long time_ns = 0;
        
        int ret = device_->writeStream(
            stream_,
            buffs,
            samples_to_write,
            flags,
            time_ns
        );

        if (ret < 0) {
            fmt::print(stderr, "[HackRFSink] writeStream error: {} ({})\n", 
                       ret, SoapySDR::errToStr(ret));
            // Не фатально, пробуем дальше
            return {0, 0};
        }

        samples_written_ += ret;
        return {static_cast<size_t>(ret), 0};
    }

    [[nodiscard]] size_t getSamplesWritten() const {
        return samples_written_;
    }

private:
    SoapySDR::Device* device_{nullptr};
    SoapySDR::Stream* stream_{nullptr};
    bool is_running_{false};
    size_t samples_written_{0};

    /**
     * @brief Применение настроек к устройству
     */
    void applySettings() {
        double cf = center_freq;
        double sr = sample_rate;
        double gain = tx_gain;
        double bw = bandwidth;

        // Проверка sample rate для HackRF (максимум 20 МГц)
        if (sr > 20e6) {
            fmt::print(stderr, "[HackRFSink] WARNING: Sample rate {} MHz exceeds HackRF max (20 MHz)\n",
                       sr / 1e6);
            fmt::print(stderr, "  Reducing to 20 MHz\n");
            sr = 20e6;
            sample_rate = sr;
        }

        // Установка параметров
        device_->setSampleRate(SOAPY_SDR_TX, 0, sr);
        device_->setFrequency(SOAPY_SDR_TX, 0, "RF", cf);
        device_->setGain(SOAPY_SDR_TX, 0, gain);
        device_->setBandwidth(SOAPY_SDR_TX, 0, bw);

        // HackRF-specific: включаем антенный порт (если поддерживается)
        if (device_->hasSetting("enable_bias")) {
            // Не включаем bias tee по умолчанию (может повредить HackRF!)
        }

        // Проверка amplifier (HackRF имеет RFFC6051 amplifier)
        if (device_->hasGainMode(SOAPY_SDR_TX, 0)) {
            device_->setGainMode(SOAPY_SDR_TX, 0, false); // Manual gain
        }
    }

    /**
     * @brief Определение формата потока на основе типа T
     */
    std::string getStreamFormat() const {
        if constexpr (std::is_same_v<T, int8_t>) {
            return "CS8";  // Complex 8-bit signed
        } else if constexpr (std::is_same_v<T, int16_t>) {
            return "CS16"; // Complex 16-bit signed
        } else if constexpr (std::is_same_v<T, float>) {
            return "CF32"; // Complex 32-bit float
        } else if constexpr (std::is_same_v<T, double>) {
            return "CF64"; // Complex 64-bit float
        } else {
            return "CS8";  // По умолчанию
        }
    }
};

GR_REGISTER_BLOCK(gr::gnss_sim::HackRFSink, (int8_t, int16_t, float, double))

} // namespace gr::gnss_sim