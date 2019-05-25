//
// Created by rj on 22.05.19.
//

#ifndef MICROMOUSE_EDGE_H
#define MICROMOUSE_EDGE_H


#include <stdint-gcc.h>

class Edge {
public:
    int getVertex1Index() const;

    void setVertex1Index(int vertex1Index);

    int getVertex2Index() const;

    void setVertex2Index(int vertex2Index);

    float getCost() const;

    void setCost(float cost);

private:
    int vertex1_index;
    int vertex2_index;
    char direction;
public:
    char getDirection() const;

    void setDirection(char direction);

private:
    float cost;

public:
    Edge(int V1, int V2, float cost, char direction);
};


#endif //MICROMOUSE_EDGE_H
