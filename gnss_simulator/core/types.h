#ifndef GNSS_SIMULATOR_H
#define GNSS_SIMULATOR_H

enum Direction {
    IN, OUT
};

class Vector3D {
public:
    double x, y, z;
    Vector3D(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0) : x(x_), y(y_), z(z_) {}
    
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }
};

class Port {
public:
    std::string name;
    Direction direction;
    
    Port(const std::string& name_, Direction direction_) : name(name_), direction(direction_) {}
    
    virtual ~Port() = default;
    virtual void push(void* data) = 0;
    virtual void* pull() = 0;
};

class MessagePort {
public:
    std::string name;
    Direction direction;
    
    MessagePort(const std::string& name_, Direction direction_) : name(name_), direction(direction_) {}
    
    virtual ~MessagePort() = default;
    virtual void send(void* msg) = 0;
    virtual void* receive() = 0;
};

class Bit {
public:
    bool value;
    Bit(bool val = true) : value(val) {}
    
    static Bit createTrue() { return Bit(true); }
    static Bit createFalse() { return Bit(false); }
    
    friend std::ostream& operator<<(std::ostream& os, const Bit& bit) {
        return os << (bit.value ? '1' : '0');
    }
};

class BitStream {
private:
    std::vector<Bit> bits;
    
public:
    BitStream() = default;
    
    int length() const { return static_cast<int>(bits.size()); }
    
    Bit getBit(int index) const {
        if (index >= 0 && index < static_cast<int>(bits.size())) {
            return bits[index];
        }
        return Bit::createFalse();
    }
    
    void append(Bit bit) {
        bits.push_back(bit);
    }
    
    std::string toString() const {
        std::string result;
        for (const auto& bit : bits) {
            result += (bit.value ? '1' : '0');
        }
        return result;
    }
};

class Block {
public:
    std::string name;
    
    Block(const std::string& name_) : name(name_) {}
    
    virtual void work() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    
    virtual ~Block() = default;
};

#endif // GNSS_SIMULATOR_H