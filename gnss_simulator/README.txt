Высокопроизводительный имитатор GNSS-сигналов на базе GNU Radio 4
Версия: 0.3.0
Лицензия: GPL-3.0
Язык: C++20
О ПРОЕКТЕ
GNSS Simulator - это современный имитатор сигналов глобальных навигационных
спутниковых систем, построенный на базе GNU Radio 4. Проект вдохновлён
архитектурой GNSS-SDR, но адаптирован для задачи генерации сигналов.
Симулятор поддерживает все основные GNSS-констелляции (GPS, GLONASS, Galileo,
BeiDou, QZSS), реалистичное моделирование атмосферы, многолучевости и помех,
а также гибкое управление через YAML-сценарии.
Применение:
Тестирование GNSS-приёмников в контролируемых условиях
Образовательные цели - изучение принципов работы GNSS
Исследования - алгоритмы обработки сигналов, навигации
Имитация городских условий - многолучевость, отражения
Тестирование устойчивости к помехам и спуфингу
КЛЮЧЕВЫЕ ВОЗМОЖНОСТИ
Мульти-констелляционность:
GPS (L1 C/A, L2C, L5)
GLONASS (L1 OF, L2 OF) с поддержкой FDMA
Galileo (E1 OS, E5A, E5B)
BeiDou (B1I, B2A)
QZSS (L1 C/A)
Кастомные системы - создайте свою ГНСС!
Реалистичное моделирование:
Точные кеплеровские орбиты (из RINEX NAV)
Ионосферные задержки (модель Klobuchar / NeQuick)
Тропосферные задержки (Saastamoinen / Hopfield)
Сдвиги часов спутников и приёмника
Доплеровский сдвиг с субчиповой точностью
Многолучевость (статические отражатели, городская среда)
Высокая производительность:
GNU Radio 4 - современный C++20 фреймворк
Zero-copy буферизация через std::span
Целочисленные фазовые аккумуляторы - без плавающей точки в критическом цикле
Compile-time LUT - таблицы синусов и кодов вычисляются компилятором
Многопоточность - автоматическое распределение блоков по ядрам CPU
SIMD-оптимизация - AVX2/AVX-512 для векторных операций
Гибкое управление:
YAML-сценарии - описание без перекомпиляции
Различные траектории - статическая, линейная, круговая, по точкам, из NMEA
Динамические параметры - изменение в реальном времени
Воспроизводимость - фиксированный seed для детерминированных тестов
Вывод сигнала:
Файловый вывод - совместимый с GNSS-SDR формат (complex float32)
HackRF One - через SoapySDR (с безопасным режимом)
USRP / LimeSDR - поддержка через SoapySDR
Логирование - подробные логи сценария и PVT
АРХИТЕКТУРА
Проект построен по модульному принципу:
YAML Scenario Parser
Парсит сценарий из YAML файла
ConstellationManager
Управление всеми спутниками (GPS, GLO, GAL, BDS, QZSS)
Расчёт видимости (elevation mask)
Приоритизация по элевации
Назначение PRN на слоты пула каналов
Channel Pool (N слотов)
GpsChannelBlock:
MultipathCodeGenerator (LUT + фазовый аккумулятор)
NCO (compile-time sine LUT)
NavBitSource (50 бит/с)
SignalCombiner
Суммирование всех активных каналов
Добавление AWGN шума (настраиваемый C/N0)
Output
FileSink (complex float32, для GNSS-SDR)
HackRFSink (через SoapySDR)
UsrpSink / LimeSdrSink (планируется)
Ключевые классы:
Satellite - Универсальный класс спутника (не зависит от ГНСС)
OrbitModel - Абстрактная орбитальная модель (стратегия)
ClockModel - Модель часов спутника (стратегия)
SignalConfig - Конфигурация сигнала (несущая, код, модуляция)
ConstellationManager - Управление созвездием и видимостью
ScenarioManager - Блок GR4, рассылающий динамику в каналы
MultipathCodeGenerator - Генератор кода с поддержкой многолучевости
NCO - Numerically Controlled Oscillator
ScenarioParser - Парсер YAML-сценариев
БЫСТРЫЙ СТАРТ
Установка зависимостей (Ubuntu/Debian):
sudo apt-get update
sudo apt-get install -y
build-essential cmake git
libfmt-dev libyaml-cpp-dev
libsoapysdr-dev soapysdr-module-hackrf
Установка GNU Radio 4 из исходников
git clone https://github.com/fair-acc/gnuradio4.git
cd gnuradio4 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j$(nproc)
sudo cmake --install .
Сборка проекта:
git clone https://github.com/yourusername/gnss-simulator.git
cd gnss-simulator
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
Запуск первого сценария:
Hello World - 10 секунд GPS сигнала в файл
./gnss_sim scenarios/hello_world.yaml
Проверка валидности сценария
./gnss_sim --validate scenarios/realistic_moscow.yaml
Dry run - показать, что будет сделано
./gnss_sim --dry-run scenarios/urban_multipath.yaml
Верификация через GNSS-SDR:
Симулятор создаёт два файла:
- output.dat (IQ данные)
- output.dat.conf (конфигурация для GNSS-SDR)
gnss-sdr --config_file=output.dat.conf
УСТАНОВКА
Системные требования:
ОС: Linux (Ubuntu 20.04+), macOS (11+)
Компилятор: GCC 11+ или Clang 14+ (с поддержкой C++20)
CMake: 3.20+
GNU Radio 4: последняя версия из git
Зависимости:
GNU Radio 4 (git master) - фреймворк обработки сигналов
fmt (10.0+) - логирование
yaml-cpp (0.7+) - парсинг YAML-сценариев
SoapySDR (0.8+) - драйвер SDR (опционально)
Сборка из исходников:
git clone https://github.com/yourusername/gnss-simulator.git
cd gnss-simulator
mkdir build && cd build
cmake ..
-DCMAKE_BUILD_TYPE=Release
-DCMAKE_INSTALL_PREFIX=/usr/local
-DENABLE_SOAPYSDR=ON
-DENABLE_PYTHON=OFF
cmake --build . -j$(nproc)
sudo cmake --install .
Опции CMake:
CMAKE_BUILD_TYPE: Debug, Release, RelWithDebInfo (по умолчанию Release)
ENABLE_SOAPYSDR: ON/OFF - включить поддержку SDR (по умолчанию ON)
ENABLE_PYTHON: ON/OFF - собрать Python bindings (по умолчанию OFF)
ENABLE_TESTS: ON/OFF - собрать unit-тесты (по умолчанию ON)
ИСПОЛЬЗОВАНИЕ
Запуск симулятора:
./gnss_sim [options] <scenario.yaml>
Опции:
--validate Только валидация сценария, без запуска
--dry-run Показать, что будет сделано
--override key=value Переопределить параметр сценария
--help Показать справку
Примеры:
Базовый запуск
./gnss_sim scenarios/hello_world.yaml
С переопределением длительности
./gnss_sim --override time.duration_sec=120 scenarios/realistic.yaml
Только проверка сценария
./gnss_sim --validate scenarios/urban.yaml
Вывод на HackRF (ТОЛЬКО В FARADAY CAGE!):
БЕЗОПАСНЫЙ режим (частота сдвинута на 1585 МГц)
./gnss_sim scenarios/hackrf_safe.yaml
Или через командную строку
./gnss_hackrf_output --safe-mode --duration 30 --gain 40
ПРИМЕРЫ СЦЕНАРИЕВ
Минимальный сценарий (Hello World):
scenarios/hello_world.yaml
metadata:
version: "1.0"
name: "Hello World"
description: "Minimal GPS simulation"
time:
duration_sec: 10.0
update_rate_hz: 10.0
receiver:
trajectory: static
position: [55.75, 37.62, 150.0] # Москва
constellation:
- system: GPS
ephemeris:
source: builtin
builtin:
num_satellites: 6
signals:
- name: L1_CA
enabled: true
output:
type: file
file:
path: "hello_world.dat"
simulation:
sample_rate_hz: 5.0e6
max_channels: 6
Реалистичный сценарий (GPS + GLONASS):
scenarios/realistic_moscow.yaml
metadata:
version: "1.0"
name: "Moscow GPS+GLO"
description: "Multi-constellation with real ephemeris"
time:
start: "2026-11-22T12:00:00Z"
duration_sec: 300.0
update_rate_hz: 10.0
receiver:
trajectory: circular
circular:
center: [55.75, 37.62, 150.0]
radius_m: 500.0
speed_mps: 5.0
elevation_mask_deg: 5.0
constellation:
- system: GPS
ephemeris:
source: rinex_nav
rinex_nav:
path: "data/brdc3260.26n"
signals:
- name: L1_CA
enabled: true
power_dbw: -130.0
 - system: GLONASS
   ephemeris:
     source: rinex_nav
     rinex_nav:
       path: "data/brdc3260.26g"
   signals:
     - name: L1_OF
       enabled: true
environment:
ionosphere:
model: klobuchar
troposphere:
model: saastamoinen
output:
type: file
file:
path: "output/moscow_gps_glo.dat"
simulation:
sample_rate_hz: 5.0e6
max_channels: 16
num_threads: 4
Городской сценарий с многолучевостью:
scenarios/urban_multipath.yaml
metadata:
version: "1.0"
name: "Urban Multipath"
description: "GPS in urban canyon"
time:
duration_sec: 120.0
receiver:
trajectory: linear
linear:
start: [55.752, 37.620, 10.0]
end: [55.754, 37.622, 10.0]
speed_mps: 2.0
constellation:
- system: GPS
ephemeris:
source: builtin
signals:
- name: L1_CA
enabled: true
environment:
multipath:
enabled: true
model: static_reflectors
static_reflectors:
- position: [55.7525, 37.6205, 30.0]
type: vertical_wall
reflection_coefficient: 0.7
phase_shift_rad: 3.14159
- position: [55.7535, 37.6215, 25.0]
type: vertical_wall
reflection_coefficient: 0.5
output:
type: file
file:
path: "output/urban_multipath.dat"
simulation:
sample_rate_hz: 5.0e6
max_channels: 12
ВЫВОД СИГНАЛА
Файловый вывод (для GNSS-SDR):
Симулятор создаёт два файла:
output.dat - бинарный файл с IQ данными (complex float32)
output.dat.conf - конфигурация для GNSS-SDR
Запуск симулятора
./gnss_sim scenarios/file_output.yaml
Верификация через GNSS-SDR
gnss-sdr --config_file=output.dat.conf
Вывод на HackRF (ТОЛЬКО В ЭКРАНИРОВАННОЙ КАМЕРЕ ИЛИ С АТТЕНЮАТОРОМ!):
scenarios/hackrf_output.yaml
output:
type: hackrf
hackrf:
center_freq_hz: 1575.42e6
sample_rate_hz: 5.0e6
tx_gain_db: 40.0
safe_mode: true # Сдвиг частоты для безопасности
Поддерживаемые SDR:
HackRF One - поддерживается (макс. 20 МГц sample rate)
USRP B200/B210 - в разработке (через SoapyUHD)
LimeSDR - в разработке (через SoapyLime)
ADALM-PLUTO - планируется (через SoapyPlutoSDR)
ЮРИДИЧЕСКОЕ ПРЕДУПРЕЖДЕНИЕ
ВНИМАНИЕ!
Передача сигналов в диапазоне GPS (1575.42 МГц) в открытое пространство
СТРОГО ЗАПРЕЩЕНА законом в большинстве стран мира!
Это создаёт помехи критической инфраструктуре (авиация, морской флот,
геодезия) и квалифицируется как уголовное преступление.
Разрешённые сценарии:
В экранированной камере (Faraday cage)
Через направленный аттенюатор с подключением напрямую к приёмнику
В лабораторных условиях с письменным разрешением
На частотах, не используемых реальными ГНСС (безопасный режим)
Использование симулятора означает ваше согласие соблюдать законодательство
вашей страны.
СТРУКТУРА ПРОЕКТА
gnss-simulator/
├── CMakeLists.txt Корневой CMake
├── README.md Этот файл
├── LICENSE GPL-3.0
│
├── include/
│ └── gnss_sim/
│ ├── core/
│ │ ├── position.hpp ECEF/геодезические координаты
│ │ └── types.hpp Базовые типы
│ │
│ ├── satellite/
│ │ ├── satellite.hpp Универсальный класс спутника
│ │ ├── satellite_state.hpp Состояние спутника
│ │ ├── orbit_model.hpp Абстрактная орбитальная модель
│ │ ├── simple_circular_orbit.hpp
│ │ ├── keplerian_orbit.hpp (планируется)
│ │ ├── clock_model.hpp Модель часов
│ │ └── signal_config.hpp Конфигурация сигнала
│ │
│ ├── constellation/
│ │ ├── constellation_manager.hpp
│ │ └── scenario_manager.hpp
│ │
│ ├── channel/
│ │ ├── gps_channel_block.hpp
│ │ ├── code_generator.hpp
│ │ ├── multipath_code_generator.hpp
│ │ ├── nco.hpp
│ │ └── nav_bit_source.hpp
│ │
│ ├── environment/
│ │ ├── ionospheric_model.hpp (планируется)
│ │ ├── tropospheric_model.hpp (планируется)
│ │ └── multipath_model.hpp (планируется)
│ │
│ ├── output/
│ │ ├── signal_combiner.hpp
│ │ ├── gnss_sdr_file_sink.hpp
│ │ ├── float_to_int8_converter.hpp
│ │ └── hackrf_sink.hpp
│ │
│ └── scenario/
│ ├── scenario_types.hpp
│ └── scenario_parser.hpp
│
├── apps/
│ ├── gnss_sim_main.cpp Главный исполняемый файл
│ ├── gnss_hello_world.cpp Hello World пример
│ └── gnss_hackrf_output.cpp Вывод на HackRF
│
├── scenarios/
│ ├── hello_world.yaml Минимальный пример
│ ├── realistic_moscow.yaml Реалистичный сценарий
│ ├── urban_multipath.yaml Городская среда
│ └── hackrf_safe.yaml Безопасный вывод на SDR
│
├── data/
│ └── (RINEX NAV файлы, эфемериды)
│
├── scripts/
│ ├── verify_output.py Верификация через корреляцию
│ └── plot_scenario.py Визуализация сценария
│
├── tests/
│ ├── test_code_generator.cpp
│ ├── test_nco.cpp
│ ├── test_satellite.cpp
│ └── test_scenario_parser.cpp
│
└── docs/
├── architecture.puml UML диаграмма
├── architecture.png
└── user_guide.md
ROADMAP
Реализовано:
Базовая архитектура на GNU Radio 4
Универсальный класс Satellite (мульти-констелляционный)
Оптимизированный генератор кода (LUT + фазовый аккумулятор)
NCO с compile-time таблицей синусов
Поддержка многолучевости
YAML-сценарии
Вывод в файл (совместимый с GNSS-SDR)
Вывод на HackRF (через SoapySDR)
ConstellationManager с пулом каналов
В разработке:
Загрузка реальных эфемерид (RINEX NAV)
KeplerianOrbit - точная орбитальная модель
Ионосферные модели (Klobuchar, NeQuick)
Тропосферные модели (Saastamoinen)
Генератор навигационных данных (GPS L1 C/A)
Траектории приёмника (все типы)
Планируется:
Поддержка USRP (через SoapyUHD)
Поддержка LimeSDR
Имитация помех (jamming, spoofing)
3D-модель города для многолучевости
GUI для мониторинга (Qt/ImGui)
Python bindings
Интеграция с GNSS-SDR (замкнутый цикл)
GPU-ускорение (CUDA/OpenCL)
ЛИЦЕНЗИЯ
Этот проект распространяется под лицензией GPL-3.0.
GNSS Simulator - High-performance GNSS signal simulator
Copyright (C) 2026 Your Name
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
КОНТРИБЬЮЦИЯ
Contributions are welcome! Если вы хотите внести вклад в проект:
Форкните репозиторий
Создайте ветку для вашей фичи (git checkout -b feature/AmazingFeature)
Закоммитьте изменения (git commit -m 'Add some AmazingFeature')
Запушьте в ветку (git push origin feature/AmazingFeature)
Откройте Pull Request
Пожалуйста, убедитесь, что ваш код:
Соответствует стилю C++20
Проходит все тесты
Документирован (Doxygen-комментарии)
Не нарушает юридических ограничений