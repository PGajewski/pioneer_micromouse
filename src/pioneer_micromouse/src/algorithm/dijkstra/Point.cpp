//
// Created by rj on 22.05.19.
//

#include "algorithm/dijkstra/Point.h"

uint16_t Point::getX() const {
    return x;
}

void Point::setX(uint16_t x) {
    Point::x = x;
}

uint16_t Point::getY() const {
    return y;
}

void Point::setY(uint16_t y) {
    Point::y = y;
}

uint32_t Point::getIndex() const {
    return index;
}

void Point::setIndex(uint32_t index) {
    Point::index = index;
}

bool Point::isRight() const {
    return is_right;
}

void Point::setIsRight(bool isRight) {
    is_right = isRight;
}

bool Point::isLeft() const {
    return is_left;
}

void Point::setIsLeft(bool isLeft) {
    is_left = isLeft;
}

bool Point::isUp() const {
    return is_up;
}

void Point::setIsUp(bool isUp) {
    is_up = isUp;
}

bool Point::isDown() const {
    return is_down;
}

void Point::setIsDown(bool isDown) {
    is_down = isDown;
}
