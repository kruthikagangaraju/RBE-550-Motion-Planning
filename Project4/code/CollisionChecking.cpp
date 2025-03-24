///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"

// TODO: Copy your implementation from previous projects


bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const auto& rect : obstacles)
    {
        if (x >= rect.x && x <= rect.x + rect.width && y >= rect.y && y <= rect.y + rect.height)
        {
            return false; 
        }
    }
    return true; 
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const auto &obs : obstacles)
    {
        double closestX = std::max(obs.x, std::min(x, obs.x + obs.width));
        double closestY = std::max(obs.y, std::min(y, obs.y + obs.height));
        
        double distanceSquared = (x - closestX) * (x - closestX) + (y - closestY) * (y - closestY);
        if (distanceSquared <= radius * radius)
        {
            return false; 
        }
    }
    return true;
}

bool intersectLine(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    double div = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    if (fabs(div) < 1e-9)
    {
        return false;
    }

    double a = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / div;
    double b = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / div;

    return (a >= 0 && a <= 1 && b >= 0 && b <= 1);
}

bool doIntersect(double p1, double q1, double p2, double q2, const Rectangle& rect)
{
    std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> rectEdges = {
        { {rect.x, rect.y}, {rect.x + rect.width, rect.y} }, 
        { {rect.x + rect.width, rect.y}, {rect.x + rect.width, rect.y + rect.height} }, 
        { {rect.x + rect.width, rect.y + rect.height}, {rect.x, rect.y + rect.height} }, 
        { {rect.x, rect.y + rect.height}, {rect.x, rect.y} } 
    };

    for (const auto& [edge1, edge2] : rectEdges)
    {
        if (intersectLine(p1, q1, p2, q2, edge1.first, edge1.second, edge2.first, edge2.second))
        {
            return true; 
        }
    }
    return false; 
}

bool PointInside(double px, double py, const std::vector<std::pair<double, double>>& polygon)
{
    bool inside = false;
    int n = polygon.size();
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        double xi = polygon[i].first, yi = polygon[i].second;
        double xj = polygon[j].first, yj = polygon[j].second;

        bool intersect = ((yi > py) != (yj > py)) && (px < (xj - xi)* (py - yi) / (yj - yi + 1e-12) + xi);
        if (intersect)
            inside = !inside;
    }
    return inside;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles, double xHighBound, double xLowBound, double yHighBound, double yLowBound)
{
    //Fillout 
    double c = cos(theta);
    double s = sin(theta);
    double halfEdge = sideLength / 2;

    std::vector<std::pair<double, double>> corners = {
        {x + halfEdge * c - halfEdge * s, y + halfEdge * s + halfEdge * c},
        {x - halfEdge * c - halfEdge * s, y - halfEdge * s + halfEdge * c},
        {x - halfEdge * c + halfEdge * s, y - halfEdge * s - halfEdge * c},
        {x + halfEdge * c + halfEdge * s, y + halfEdge * s - halfEdge * c}
    };

    double minX = corners[0].first;
    double maxX = corners[0].first;
    double minY = corners[0].second;
    double maxY = corners[0].second;
    for (const auto& [cx, cy] : corners)
    {
        minX = std::min(minX, cx);
        maxX = std::max(maxX, cx);
        minY = std::min(minY, cy);
        maxY = std::max(maxY, cy);
    }

    if (minX < xLowBound || maxX > xHighBound || minY < yLowBound || maxY > yHighBound)
    {
        return false;
    }

    for (const auto& obstacle : obstacles)
    {
        bool cornerInside = false;
        for (const auto& [cx, cy] : corners) 
        {
            if (cx >= obstacle.x && cx <= obstacle.x + obstacle.width &&
                cy >= obstacle.y && cy <= obstacle.y + obstacle.height) 
            {
                return false; 
            }
        }

        for (int i = 0; i < 4; ++i)
        {
            int j = (i + 1) % 4;
            if (doIntersect(corners[i].first, corners[i].second,
                corners[j].first, corners[j].second, obstacle))
            {
                return false; 
            }
        }

        std::vector<std::pair<double, double>> obstacleCorners = {
            {obstacle.x, obstacle.y},
            {obstacle.x + obstacle.width, obstacle.y},
            {obstacle.x + obstacle.width, obstacle.y + obstacle.height},
            {obstacle.x, obstacle.y + obstacle.height}
        };

        for (const auto& [ox, oy] : obstacleCorners) 
        {
            if (PointInside(ox, oy, corners)) 
            {
                return false; 
            }
        }
    }

    return true; 
}
