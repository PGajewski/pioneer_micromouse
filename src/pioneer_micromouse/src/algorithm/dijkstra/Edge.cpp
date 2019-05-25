//
// Created by rj on 22.05.19.
//

#include "algorithm/dijkstra/Edge.h"

Edge::Edge(int V1, int V2, float cost, char direction) {
    this->vertex1_index = V1;
    this->vertex2_index = V2;
    this->cost = cost;
    this->direction = direction;
}

int Edge::getVertex1Index() const {
    return vertex1_index;
}

void Edge::setVertex1Index(int vertex1Index) {
    vertex1_index = vertex1Index;
}

int Edge::getVertex2Index() const {
    return vertex2_index;
}

void Edge::setVertex2Index(int vertex2Index) {
    vertex2_index = vertex2Index;
}

float Edge::getCost() const {
    return cost;
}

void Edge::setCost(float cost) {
    Edge::cost = cost;
}

char Edge::getDirection() const {
    return direction;
}

void Edge::setDirection(char direction) {
    Edge::direction = direction;
}
