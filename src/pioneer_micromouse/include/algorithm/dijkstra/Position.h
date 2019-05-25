//
// Created by rj on 23.05.19.
//

#ifndef MICROMOUSE_POSITION_H
#define MICROMOUSE_POSITION_H


#include <tiff.h>

class Position {
public:
    Position();

    int prev;

    Position(char dir, int index, int prev, float cost);

    float getCost() const;

    void setCost(float cost);

    char getDirection() const;

    void setDirection(char direction);

private:
    int index;
public:
    int getIndex() const;

    void setIndex(int index);

private:
    float cost;
    char direction;
};


#endif //MICROMOUSE_POSITION_H
