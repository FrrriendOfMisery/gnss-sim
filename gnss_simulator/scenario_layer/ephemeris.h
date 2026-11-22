#ifndef EPHemeris_H
#define EPHemeris_H

#include "gnss_simulator/core/types.h"
#include <iostream>

class Ephemeris {
public:
    int svId = 0;
    double toe = 0.0;
    double sqrtA = 1.0; // Semi-major axis
    double e = 0.0;     // Eccentricity
    double omega0 = 0.0;// Argument of perigee at reference time
    double i0 = 0.0;    // Inclination
    double omega = 0.0; // Rate of argument of perigee
    
    Ephemeris(int svId_ = 0) : svId(svId_) {}
    
    Vector3D calculatePosition(double time) {
        // Упрощенный расчет позиции спутника по Keplерian элементам
        // В реальном коде здесь были бы полные уравнения
        
        double M = M0 + omega * (time - toe);
        // Здесь нужно было бы вычислить E из Kepler equation: M = E - e*sinE
        // Затем найти true anomaly f и mean anomaly
        
        return Vector3D(1.0, 0.0, 0.0); // Временный результат
    }
    
    Vector3D calculateVelocity(double time) {
        // Упрощенный расчет скорости спутника
        double angularVelocity = std::sqrt(GM / sqrtA * (4 / (sqrt(1 - e*e))));
        
        return Vector3D(-sqrtA * angularVelocity * std::sin(M0), 
                        sqrtA * angularVelocity * std::cos(M0), 
                        0.0);
    }
    
    int getSVId() const { return svId; }
};

#endif // EPHemeris_H