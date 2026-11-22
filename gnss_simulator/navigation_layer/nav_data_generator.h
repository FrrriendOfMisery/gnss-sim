#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <array>
#include <vector>
#include <cmath>

namespace gr::gnss {

// Структура для хранения эфемерид (упрощенно)
struct GpsEphemeris {
    int prn;
    double toe;      // Time of Ephemeris
    // ... остальные параметры (sqrtA, e, i0, omega0, etc.)
    std::array<uint32_t, 10> subframe_words; // Предварительно рассчитанные слова
};

template <typename T>
struct GpsNavMessageGenerator : public gr::Block<GpsNavMessageGenerator<T>> {
    
    // --- Порты ---
    // Тактовый вход (например, 50 Hz триггер или время)
    gr::PortIn<T> time_in;
    // Выходной поток бит (50 bps)
    gr::PortOut<uint8_t> nav_bits_out;
    
    // Асинхронный порт для приема эфемерид
    gr::MessagePort ephemeris_in;

    // --- Свойства (настраиваются из flowgraph) ---
    GR_PROPERTY(int, sv_prn) = 1;
    GR_PROPERTY(double, sample_rate) = 50.0; // Жестко 50 Гц для GPS L1 C/A

    // --- Внутреннее состояние ---
    GpsEphemeris current_ephemeris_;
    std::array<uint8_t, 300> subframe_buffer_{}; // Буфер подкадра (300 бит)
    int current_bit_index_ = 0;
    uint32_t last_parity_bits_ = 0; // Для расчета четности (D29*, D30*)
    
    // Флаг, что D30* из предыдущего слова (инвертирование следующего слова)
    bool invert_next_word_ = false; 

    GpsNavMessageGenerator(gr::property_map props = {}) 
        : gr::Block<GpsNavMessageGenerator<T>>(std::move(props)) {
        
        // Регистрация обработчика сообщений
        this->register_handler("ephemeris_in", [this](const gr::Message& msg) {
            this->handle_ephemeris_update(msg);
        });
    }

    // --- Обработка сообщений ---
    void handle_ephemeris_update(const gr::Message& msg) {
        // Десериализация PMT в структуру GpsEphemeris
        // current_ephemeris_ = parse_pmt_to_ephemeris(msg.data);
        // generate_subframe_bits();
    }

    // --- Основной алгоритм (Work function) ---
    gr::work::Result work(gr::work::IOContext& ctx) {
        auto in_samples = time_in.streamBuffer();
        auto out_samples = nav_bits_out.streamBuffer();

        if (in_samples.empty() || out_samples.empty()) {
            return gr::work::Result::INSUFFICIENT_INPUT;
        }

        size_t n_to_process = std::min(in_samples.size(), out_samples.size());
        
        for (size_t i = 0; i < n_to_process; ++i) {
            // 1. Получаем время (или просто используем такт)
            // T current_time = in_samples[i]; 
            
            // 2. Достаем следующий бит из буфера подкадра
            uint8_t bit = subframe_buffer_[current_bit_index_];
            
            // 3. Применяем инверсию, если D30* предыдущего слова был 1
            if (invert_next_word_) {
                bit ^= 1; 
            }
            
            out_samples[i] = bit;
            
            // 4. Обновляем индексы и состояние
            current_bit_index_++;
            if (current_bit_index_ >= 300) {
                current_bit_index_ = 0;
                // Переход к следующему подкадру, генерация новых 300 бит
                generate_next_subframe(); 
            }
        }

        time_in.consume(n_to_process);
        nav_bits_out.produce(n_to_process);

        return gr::work::Result::OK;
    }

private:
    // Генерация 300 бит подкадра (10 слов по 30 бит)
    void generate_next_subframe() {
        // Логика заполнения subframe_buffer_
        // Для каждого из 10 слов:
        // 1. Берем 24 бита данных (из эфемерид или TLM/HOW)
        // 2. Считаем 6 бит четности (Parity) на основе данных и last_parity_bits_
        // 3. Упаковываем 30 бит в subframe_buffer_
        // 4. Сохраняем последние 2 бита (D29*, D30*) для инверсии следующего слова
        
        update_parity_state();
    }

    void update_parity_state() {
        // Реализация алгоритма проверки четности GPS (24 бита данных -> 6 бит четности)
        // См. ICD-GPS-200, Table 20-XIV
    }
};

} // namespace gr::gnss