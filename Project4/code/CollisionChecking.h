#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);

bool intersectLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);

bool doIntersect(double x1, double y1, double x2, double y2, const Rectangle& rect);

bool PointInside(double px, double py, const std::vector<std::pair<double, double>>& polygon);

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles, double xHighBound, double xLowBound, double yHighBound, double yLowBound);

#endif
