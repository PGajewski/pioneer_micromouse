//
// Created by rj on 22.05.19.
//

#ifndef MICROMOUSE_POINT_H
#define MICROMOUSE_POINT_H


#include <stdint-gcc.h>

class Point {
public:

    uint16_t getX() const;

    void setX(uint16_t x);

    uint16_t getY() const;

    void setY(uint16_t y);

    uint32_t getIndex() const;

    void setIndex(uint32_t index);

    bool isRight() const;

    void setIsRight(bool isRight);

    bool isLeft() const;

    void setIsLeft(bool isLeft);

    bool isUp() const;

    void setIsUp(bool isUp);

    bool isDown() const;

    void setIsDown(bool isDown);

    Point(){
        this->x = 0;
        this->y = 0;
        this->index = 0;
        this->is_down = false;
        this->is_up = false;
        this->is_left = false;
        this->is_right = false;
    }

private:
    uint16_t x;
    uint16_t y;
    uint32_t index;
    bool is_right;
    bool is_left;
    bool is_up;
    bool is_down;
};


#endif //MICROMOUSE_POINT_H
