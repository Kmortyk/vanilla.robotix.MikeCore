//
// Created by eldar on 23.07.2020.
//

#ifndef SRC_ASTAR_HPP
#define SRC_ASTAR_HPP

#include <vector>
#include <cmath>
#include <stack>
#include <utility>
#include <iostream>
#include <bits/stdc++.h>
#include <set>

// 1 is free cell

class AStar {

public:
    void test();

    struct Point2DInt {
        int x, y;
    };

    std::stack<Point2DInt> search(int iFrom, int jFrom, int iDestination, int jDestination);

    std::stack<AStar::Point2DInt> nullStack;

    std::map<int, std::map<int, int8_t>> map;

    int iMax;
    int jMax;

private:

    //std::vector<std::vector<double>> map;

    struct cell {
        int iParent, jParent;
        double f, g, h; // f = g + h
    };

    bool ijIsValid(int i, int j) const;

    bool ijIsUnblocked(int i, int j) const;

    static bool ijIsDestination(int i, int j, int iDestination, int jDestination);

    static double calculateHeuristicValue(int i, int j, int iDestination, int jDestination);

    static void tracePath(std::map<int, std::map<int, cell>> cells, int iDestination, int jDestination);

    static std::stack<Point2DInt> getPath(std::map<int, std::map<int, cell>> cells, int iDestination, int jDestination);



};

#endif //SRC_ASTAR_HPP
