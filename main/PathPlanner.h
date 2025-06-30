#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <Arduino.h>
#include <vector>

struct Point {
    float x;
    float y;
};

struct Pose2d {
    float x;
    float y;
    float h;
};

class PathPlanner {
    public:
        static std::vector<Point> generateSpline(std::vector<Point>* controlPoints);
        static float catmullRom(float t, float p0, float p1, float p2, float p3);

};
#endif // PATHPLANNER_H