#ifndef NAV_MESSAGE_H
#define NAV_MESSAGE_H

#include "gnss_simulator/core/types.h"

class NavMessage {
public:
    int svId = 0;
    int subframe = 0;
    std::vector<Bit> words(30); // Массив из 30 бит
    
    NavMessage(int svId_ = 0) : svId(svId_) {}
    
    Bit calculateParity() {
        // Вычисление контрольной суммы (parity bits)
        int paritySum = 0;
        for (const auto& word : words) {
            if (word.value) {
                paritySum++;
            }
        }
        return Bit(paritySum % 2);
    }
    
    BitStream encode() const {
        // Кодирование навигационного сообщения
        BitStream encoded;
        for (const auto& word : words) {
            if (word.value) {
                encoded.append(Bit::createTrue());
            } else {
                encoded.append(Bit::createFalse());
            }
        }
        return encoded;
    }
    
private:
    void updateWord(int index, bool value) {
        if (index >= 0 && index < static_cast<int>(words.size())) {
            words[index].value = value;
        }
    }
};

#endif // NAV_MESSAGE_H