//
// Created by rj on 23.05.19.
//

#include "algorithm/dijkstra/Ver.h"

Ver::Ver(uint32_t index, float cost, char direction) {
    this->index = index;
    this->cost = cost;
    this->direction = direction;
}
Ver::Ver(){}

uint32_t Ver::getIndex() const {
    return index;
}

void Ver::setIndex(uint32_t index) {
    Ver::index = index;
}

float Ver::getCost() const {
    return cost;
}

void Ver::setCost(float cost) {
    Ver::cost = cost;
}

char Ver::getDirection() const {
    return direction;
}

void Ver::setDirection(char direction) {
    Ver::direction = direction;
}
