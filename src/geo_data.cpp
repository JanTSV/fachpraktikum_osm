#include "geo_data.hpp"

#include <cmath>

Point::Point() : x(0), y(0) { }

Point::Point(double x_, double y_) : x(x_), y(y_) { }

double Point::euclidean_distance(Point& other) {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
}

Building::Building() : location(), street_idx(std::nullopt), house_number(std::nullopt) { }

Building::Building(Point location_, std::optional<size_t> street_idx_, std::optional<HouseNumber> house_number_)
    : location(location_), street_idx(street_idx_), house_number(house_number_) { }

Street::Street() : name_idx(0), points() { }

Street::Street(size_t name_idx_, std::vector<Point> points_)
    : name_idx(name_idx_), points(points_) { }

AdminArea::AdminArea() : name_idx(0), boundary(), level(0) { }

AdminArea::AdminArea(size_t name_idx_, std::vector<Point> boundary_, uint8_t level_)
    : name_idx(name_idx_), boundary(boundary_), level(level_) { }