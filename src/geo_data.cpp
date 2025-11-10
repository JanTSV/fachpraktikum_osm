#include "geo_data.hpp"

#include <cmath>

Point::Point() : x(0), y(0) { }

Point::Point(double x_, double y_) : x(x_), y(y_) { }

double& Point::operator[](size_t idx) {
    if (idx == 0) return x;
    else if (idx == 1) return y;
    else throw std::out_of_range("Point index out of range");
}

const double& Point::operator[](size_t idx) const {
    if (idx == 0) return x;
    else if (idx == 1) return y;
    else throw std::out_of_range("Point index out of range");
}

double Point::euclidean_distance(const Point& other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
}

Building::Building() : location(), street_idx(std::nullopt), house_number(std::nullopt) { }

Building::Building(Point location, std::optional<size_t> street_idx, std::optional<HouseNumber> house_number)
    : location(location), street_idx(street_idx), house_number(house_number) { }

Street::Street() : name_idx(0), points() { }

Street::Street(size_t name_idx, std::vector<Point> points)
    : name_idx(name_idx), points(points) { }

AdminArea::AdminArea() : name_idx(0), boundary(), level(0) { }

AdminArea::AdminArea(size_t name_idx, std::vector<Point> boundary, uint8_t level)
    : name_idx(name_idx), boundary(boundary), level(level) { }