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

Point Point::project_mercator(double lat, double lon) {
    double x = R * lon * M_PI / 180.0;

    double lat_rad = lat * M_PI / 180.0;
    constexpr double max_lat_rad = 1.48352986419518;
    if (lat_rad > max_lat_rad) lat_rad = max_lat_rad;
    if (lat_rad < -max_lat_rad) lat_rad = -max_lat_rad;

    double y = R * std::log(std::tan(M_PI / 4.0 + lat_rad / 2.0));
    return Point(x, y);
}


Building::Building() : location(), street_idx(std::nullopt), house_number(std::nullopt), address(0) { }

Building::Building(Point location, std::optional<size_t> street_idx, std::optional<HouseNumber> house_number)
    : location(location), street_idx(street_idx), house_number(house_number), address(0) { }

Street::Street() : name_idx(0), points() { }

Street::Street(size_t name_idx, std::vector<Point> points)
    : name_idx(name_idx), points(points) { }

AdminArea::AdminArea() : name_idx(0), boundary(), level(0), bl(Point()), tr(Point()), _projected_boundary() { }

AdminArea::AdminArea(size_t name_idx, std::vector<Point> boundary, uint8_t level)
    : name_idx(name_idx), boundary(boundary), level(level) {
    
    if (this->boundary.empty()) {
        bl = tr = Point(0, 0);
        return;
    }

    _projected_boundary.clear();
    _projected_boundary.reserve(boundary.size());

    bl = Point::project_mercator(this->boundary[0].x, this->boundary[0].y);
    tr = Point::project_mercator(this->boundary[0].x, this->boundary[0].y);

    for (const auto& p : this->boundary) {
        const Point projected = Point::project_mercator(p.x, p.y);
        if (projected.x < bl.x) bl.x = projected.x;
        if (projected.y < bl.y) bl.y = projected.y;
        if (projected.x > tr.x) tr.x = projected.x;
        if (projected.y > tr.y) tr.y = projected.y;
        _projected_boundary.push_back(projected);
    }
}

bool AdminArea::point_in_polygon(const Point& p) const {
    const Point projected = Point::project_mercator(p.x, p.y);
    if (projected.x < bl.x || projected.x > tr.x || projected.y < bl.y || projected.y > tr.y) return false;

     bool inside = false;
     const size_t n = _projected_boundary.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point pi = _projected_boundary[i];
        const Point pj = _projected_boundary[j];
        if (((pi.y > projected.y) != (pj.y > projected.y)) &&
            (projected.x < (pj.x - pi.x) * (projected.y - pi.y) / (pj.y - pi.y) + pi.x))
        {
            inside = !inside;
        }
    }

    return inside;
}