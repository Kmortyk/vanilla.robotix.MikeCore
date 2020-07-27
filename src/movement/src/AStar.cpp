//
// Created by eldar on 23.07.2020.
//

#include "AStar.hpp"

bool AStar::ijIsValid(int i, int j) const {
    return i >= 0 && i < iMax && j >= 0 && j < jMax;
}

bool AStar::ijIsUnblocked(int i, int j) const {
    return map[i][j] == 1;
}

bool AStar::ijIsDestination(int i, int j, int iDestination, int jDestination) {
    return i == iDestination && j == jDestination;
}

double AStar::calculateHeuristicValue(int i, int j, int iDestination, int jDestination) {
    return std::sqrt((i - iDestination) * (i - iDestination) + (j - jDestination) * (j - jDestination));
}

void AStar::tracePath(std::map<int, std::map<int, cell>> cells, int iDestination, int jDestination) {
    std::cout << std::endl << "The path is ";
    int i = iDestination;
    int j = jDestination;
    std::stack<std::pair<int, int>> path;
    while (!(cells[i][j].iParent == i && cells[i][j].jParent == j)) {
        path.push(std::make_pair(i, j));
        int iTemp = cells[i][j].iParent;
        int jTemp = cells[i][j].jParent;
        i = iTemp;
        j = jTemp;
    }
    path.push(std::make_pair(i, j));
    while (!path.empty()) {
        std::pair<int, int> p = path.top();
        path.pop();
        std::cout << "-> (" << p.first << ',' << p.second << ") ";
    }
    std::cout << std::endl;
}

void AStar::search(int iFrom, int jFrom, int iDestination, int jDestination) {
    if (!ijIsValid(iFrom, jFrom)) {
        std::cerr << "Source i, j is invalid!" << std::endl;
        return;
    }
    if (!ijIsValid(iDestination, jDestination)) {
        std::cerr << "Destination i, j is invalid!" << std::endl;
        return;
    }
    if (!(ijIsUnblocked(iFrom, jFrom)) || !(ijIsUnblocked(iDestination, jDestination))) {
        std::cerr << "Source or the destination is blocked" << std::endl;
    }
    if (ijIsDestination(iFrom, jFrom, iDestination, jDestination)) {
        std::cerr << "We are already at the destination!" << std::endl;
        return;
    }
    std::map<int, std::map<int, bool>> closedList;
    std::map<int, std::map<int, cell>> cells;
    for (int i = 0; i < iMax; ++i) {
        for (int j = 0; j < jMax; ++j) {
            cells[i][j].f = FLT_MAX;
            cells[i][j].g = FLT_MAX;
            cells[i][j].h = FLT_MAX;
            cells[i][j].iParent = -1;
            cells[i][j].jParent = -1;
        }
    }
    cells[iFrom][jFrom].f = 0.0;
    cells[iFrom][jFrom].g = 0.0;
    cells[iFrom][jFrom].h = 0.0;
    cells[iFrom][jFrom].iParent = iFrom;
    cells[iFrom][jFrom].jParent = jFrom;

    std::set<std::pair<double, std::pair<int, int>>> openList;
    openList.insert(std::make_pair(0, std::make_pair(iFrom, jFrom)));
    bool foundDestination = false;

    while (!openList.empty()) {
        std::pair<double, std::pair<int, int>> p = *openList.begin();
        openList.erase(openList.begin());
        int i = p.second.first;
        int j = p.second.second;
        closedList[i][j] = true;

        struct successorVariables {
            int i, j;
            double additionalFactor;
        };

        successorVariables allVarious[] = {
                {i - 1, j, 1}, // North
                {i + 1, j, 1}, // South
                {i, j + 1, 1}, // East
                {i, j - 1, 1}, // West
                {i - 1, j + 1, 1.414}, // North-East
                {i - 1, j - 1, 1.414}, // North-West
                {i + 1, j + 1, 1.414}, // South-East
                {i + 1, j - 1, 1.414}, // South-West
        };
        double gNew, hNew, fNew;
        int counter = -1;

        calculateSuccessor:
        counter++;
        int iCurrent = allVarious[counter].i;
        int jCurrent = allVarious[counter].j;
        double additionalFactor = allVarious[counter].additionalFactor;
        if (ijIsValid(iCurrent, jCurrent)) {
            if (ijIsDestination(iCurrent, jCurrent, iDestination, jDestination)) {
                cells[iCurrent][jCurrent].iParent = i;
                cells[iCurrent][jCurrent].jParent = j;
                std::cout << "The destination cell is found" << std::endl;
                tracePath(cells, iDestination, jDestination);
                foundDestination = true;
                return;
            } else if (!(closedList[iCurrent][jCurrent]) && ijIsUnblocked(iCurrent, jCurrent)) {
                gNew = cells[i][j].g + additionalFactor;
                hNew = calculateHeuristicValue(iCurrent, jCurrent, iDestination, jDestination);
                fNew = gNew + hNew;
                if (cells[iCurrent][jCurrent].f == FLT_MAX || cells[iCurrent][jCurrent].f > fNew) {
                    openList.insert(std::make_pair(fNew, std::make_pair(iCurrent, jCurrent)));
                    cells[iCurrent][jCurrent].f = fNew;
                    cells[iCurrent][jCurrent].g = gNew;
                    cells[iCurrent][jCurrent].h = hNew;
                    cells[iCurrent][jCurrent].iParent = i;
                    cells[iCurrent][jCurrent].jParent = j;
                }
            }
        }
        if (counter < 7)
            goto calculateSuccessor;
    }
    if (!foundDestination)
        std::cerr << "Failed to find the destination!" << std::endl;
}

void AStar::test() {
    iMax = 9;
    jMax = 10;
    map = {
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
            { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
            { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
    };
    search(8, 0, 0, 0);
}
