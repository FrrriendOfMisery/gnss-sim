cmake_minimum_required(VERSION 3.20)
project(gr-gnss-sim VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Зависимости
find_package(gnuradio REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPYSDR SoapySDR)
find_package(yaml-cpp REQUIRED)

# Библиотека
add_library(gnss-sim INTERFACE)
target_include_directories(gnss-sim INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_link_libraries(gnss-sim INTERFACE
    gnuradio::core
    fmt::fmt
    yaml-cpp::yaml-cpp
)

if(SOAPYSDR_FOUND)
    target_link_libraries(gnss-sim INTERFACE ${SOAPYSDR_LIBRARIES})
    target_compile_definitions(gnss-sim INTERFACE HAS_SOAPYSDR)
endif()

# Исполняемые файлы
add_executable(gnss_sim apps/gnss_sim_main.cpp)
target_link_libraries(gnss_sim PRIVATE gnss-sim)

# Копируем примеры сценариев
file(COPY scenarios DESTINATION ${CMAKE_BINARY_DIR})