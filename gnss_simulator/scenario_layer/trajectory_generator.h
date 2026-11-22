#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "gnss_simulator/core/types.h"

class TrajectoryGenerator {
public:
    Vector3D startPosition;
    Vector3D velocity;
    
    explicit TrajectoryGenerator(const Vector3D& start, const Vector3D& vel) 
        : startPosition(start), velocity(vel) {}
    
    Vector3D generatePosition(double time) {
        // Генерация позиции по линейной траектории (упрощенная модель)
        return startPosition + velocity * time;
    }
    
    void setStartPosition(const Vector3D& pos) {
        startPosition = pos;
    }
    
    void setVelocity(const Vector3D& vel) {
        velocity = vel;
    }
};

#endif // TRAJECTORY_GENERATOR_H