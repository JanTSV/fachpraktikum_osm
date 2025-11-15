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

AdminArea::AdminArea() : name_idx(0), boundary(), level(0), _bl(Point()), _tr(Point()) { }

AdminArea::AdminArea(size_t name_idx, std::vector<Point> boundary, uint8_t level)
    : name_idx(name_idx), boundary(boundary), level(level) {
    
    if (this->boundary.empty()) {
        _bl = _tr = Point(0, 0);
        return;
    }

    double min_x = this->boundary[0].x;
    double min_y = this->boundary[0].y;
    double max_x = this->boundary[0].x;
    double max_y = this->boundary[0].y;

    for (const auto& p : this->boundary) {
        if (p.x < min_x) min_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.x > max_x) max_x = p.x;
        if (p.y > max_y) max_y = p.y;
    }

    _bl = Point(min_x, min_y);
    _tr = Point(max_x, max_y);
}

bool AdminArea::point_in_polygon(const Point& p) const {
    if (p.x < _bl.x || p.x > _tr.x || p.y < _bl.y || p.y > _tr.y) return false;

     bool inside = false;
     const size_t n = boundary.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& pi = boundary[i];
        const Point& pj = boundary[j];
        if (((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x))
        {
            inside = !inside;
        }
    }

    return inside;
}