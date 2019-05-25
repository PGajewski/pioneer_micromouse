//
// Created by rj on 23.05.19.
//

#ifndef MICROMOUSE_VER_H
#define MICROMOUSE_VER_H


#include <cstdint>

class Ver {
public:
    Ver(uint32_t index, float cost, char direction);
    Ver();
private:
public:
    uint32_t getIndex() const;

    void setIndex(uint32_t index);

    float getCost() const;

    void setCost(float cost);

    char getDirection() const;

    void setDirection(char direction);

private:
    uint32_t index;
    float cost;
    char direction;
};

#endif //MICROMOUSE_VER_H
