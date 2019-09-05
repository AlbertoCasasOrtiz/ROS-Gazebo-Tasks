//
// Created by alberto on 5/09/19.
//

#ifndef SRC_POINT_H
#define SRC_POINT_H

#include "math.h"

class Point {
public:
    /// X coordinate of the point.
    float x;
    /// Y coordinate of the point.
    float y;

    /// Calculate euclidean distance between two points.
    /// \param point1 Point to calculate.
    /// \param point2 Point to calculate.
    /// \return Euclidean distance between two points.
    static float euclideanDistance(Point point1, Point point2){
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.x - point2.y, 2));
    }

    /// Calculate manhattan distance between two points.
    /// \param point1 Point to calculate.
    /// \param point2 Point to calculate.
    /// \return Manhattan distance between two points.
    static float manhattanDistance(Point point1, Point point2){
        return fabs(point1.x - point2.x) + fabs(point1.y + point2.y);
    }

};


#endif //SRC_POINT_H
