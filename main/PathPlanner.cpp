// #include <PathPlanner.h>

// std::vector<Point> PathPlanner::generateSpline(std::vector<Point>* controlPoints) {
//     float step = 0.05;
//     int numValues = controlPoints->size();
//     std::vector<Point> splinePoints = {};

//     for (int i = 1; i < numValues - 2; i++) {
//         Point p0 = controlPoints->at(i-1);
//         Point p1 = controlPoints->at(i);
//         Point p2 = controlPoints->at(i+1);
//         Point p3 = controlPoints->at(i+2);

//         for (float t = 0; t <= 1.0; t += step) {
//             if (splinePoints.size() < 200) {
//                 float x = catmullRom(t, p0.x, p1.x, p2.x, p3.x);
//                 float y = catmullRom(t, p0.y, p1.y, p2.y, p3.y);
//                 splinePoints.push_back({x, y}); 
//             }
//         }
//     }

//     return splinePoints;
// }

// float PathPlanner::catmullRom(float t, float p0, float p1, float p2, float p3) {
//     float t2 = t * t;
//     float t3 = t2 * t;
//     return 0.5 * (
//         (2 * p1) +
//         (-p0 + p2) * t +
//         (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
//         (-p0 + 3*p1 - 3*p2 + p3) * t3
//   );
// }
