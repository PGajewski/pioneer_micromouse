//
// Created by rj on 23.05.19.
//

#include "algorithm/dijkstra/Position.h"

float Position::getCost() const {
    return cost;
}

void Position::setCost(float cost) {
    Position::cost = cost;
}

char Position::getDirection() const {
    return direction;
}

void Position::setDirection(char direction) {
    Position::direction = direction;
}

Position::Position() {
    this->cost = 1000;
}

Position::Position(char dir, int index, int prev, float cost=1000) {
    this->cost = cost;
    this->direction = dir;
    this->index = index;
    this->prev = prev;
}

int Position::getIndex() const {
    return index;
}

void Position::setIndex(int index) {
    Position::index = index;
}
