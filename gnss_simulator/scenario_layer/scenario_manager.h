#ifndef SCENARIO_MANAGER_H
#define SCENARIO_MANAGER_H

#include "gnss_simulator/core/types.h"
#include <vector>
#include <string>

class Ephemeris;
class SatelliteDynamics;
class TrajectoryGenerator;

// Forward declarations
struct EphemerisData {
    int svId;
    double toe;
    double sqrtA;
    double e;
    double omega0;
    double i0;
    double omega;
    double M0;
};

struct SatelliteDynamicsData {
    int svId;
    double codePhase;
    double carrierPhase;
    double dopplerFreq;
    double elevation;
    double azimuth;
    double pseudorange;
};

class ScenarioManager : public Block {
public:
    std::vector<EphemerisData> ephemerisData; // Используем структуры данных
    std::vector<SatelliteDynamicsData> dynamicsData;
    double currentTime = 0.0;
    
    explicit ScenarioManager(const std::string& name) : Block(name) {}
    
    void loadScenario(const std::string& filename) {
        // Чтение сценария из файла или базы данных
        // Здесь должна быть логика загрузки ephemeris и dynamics данных
    }
    
    void updateDynamics(double time) {
        currentTime = time;
        // Обновление динамики для всех спутников
        for (auto& dyn : dynamicsData) {
            calculateAndUpdateDynamics(dyn, time);
        }
    }
    
    SatelliteDynamicsData* getDynamics(int svId) {
        for (auto& dyn : dynamicsData) {
            if (dyn.svId == svId) {
                return &dyn;
            }
        }
        return nullptr;
    }
    
private:
    void calculateAndUpdateDynamics(SatelliteDynamicsData& dyn, double time) {
        // Расчет новой позиции и скорости спутника
        // Обновление фазы кода и несущей
        double position = getPositionFromEphemeris(dyn.svId, time);
        double velocity = getVelocityFromEphemeris(dyn.svId, time);
        // Применение к динамике спутника
    }
    
    Vector3D calculatePosition(int svId, double time) {
        // Получение данных эфемерид для данного SV и времени
        // Расчет позиции по Keplerian орбите
        return Vector3D();
    }
};

#endif // SCENARIO_MANAGER_H