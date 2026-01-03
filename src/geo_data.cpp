#include "geo_data.hpp"

#include <cmath>
#include <cfloat>
#include <algorithm>

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

const double EARTH_RADIUS_M = 6371000.0;

double Point::haversine_distance(const Point& other) const {
    double lat1 = x * M_PI / 180.0;
    double lon1 = y * M_PI / 180.0;
    double lat2 = other.x * M_PI / 180.0;
    double lon2 = other.y * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double sin_dlat_2 = std::sin(dlat / 2.0);
    double sin_dlon_2 = std::sin(dlon / 2.0);

    double a_hav = sin_dlat_2 * sin_dlat_2 +
                   std::cos(lat1) * std::cos(lat2) * sin_dlon_2 * sin_dlon_2;

    double c = 2.0 * std::atan2(std::sqrt(a_hav), std::sqrt(1.0 - a_hav));

    return EARTH_RADIUS_M * c;
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

AdminArea::AdminArea() :
    name_idx(0), boundaries(), bl(Point()), tr(Point()), level(0), _projected_boundaries(), _edges(), _bins(), _bin_heights() { }

AdminArea::AdminArea(size_t name_idx,
                     std::vector<std::vector<Point>> boundaries,
                     uint8_t level)
    : name_idx(name_idx),
      boundaries(std::move(boundaries)),
      level(level)
{
    constexpr size_t BIN_COUNT = 32;

    if (this->boundaries.empty()) {
        bl = tr = Point(0.0, 0.0);
        return;
    }

    bool bbox_initialized = false;

    _projected_boundaries.clear();
    _edges.clear();
    _bins.clear();
    _bin_heights.clear();

    _projected_boundaries.reserve(this->boundaries.size());
    _edges.reserve(this->boundaries.size());
    _bins.reserve(this->boundaries.size());
    _bin_heights.reserve(this->boundaries.size());

    // Project all coordinates of boundary rings
    for (const auto& ring : this->boundaries) {
        if (ring.size() < 3) continue;

        std::vector<Point> projected;
        projected.reserve(ring.size());

        for (const auto& p : ring) {
            Point mp = Point::project_mercator(p.x, p.y);

            if (!bbox_initialized) {
                bl = tr = mp;
                bbox_initialized = true;
            } else {
                bl.x = std::min(bl.x, mp.x);
                bl.y = std::min(bl.y, mp.y);
                tr.x = std::max(tr.x, mp.x);
                tr.y = std::max(tr.y, mp.y);
            }

            projected.push_back(mp);
        }

        _projected_boundaries.push_back(std::move(projected));
    }

    // Compute all edges per boundary ring
    for (const auto& ring : _projected_boundaries) {
        std::vector<Edge> ring_edges;
        ring_edges.reserve(ring.size());

        const size_t n = ring.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const Point& pi = ring[i];
            const Point& pj = ring[j];
            if (pi.y == pj.y) continue;

            Edge e;
            if (pi.y < pj.y) {
                e.y_min = pi.y;
                e.y_max = pj.y;
                e.x_at_y_min = pi.x;
                e.inv_slope = (pj.x - pi.x) / (pj.y - pi.y);
            } else {
                e.y_min = pj.y;
                e.y_max = pi.y;
                e.x_at_y_min = pj.x;
                e.inv_slope = (pi.x - pj.x) / (pi.y - pj.y);
            }

            ring_edges.push_back(e);
        }

        // Calculate all bins per boundary ring
        double bin_height = (tr.y - bl.y) / BIN_COUNT;
        if (bin_height < 1e-12) bin_height = 1.0;

        std::vector<std::vector<size_t>> ring_bins(BIN_COUNT);

        auto clamp_to_bin = [&](double y) {
            size_t b = size_t((y - bl.y) / bin_height);
            if (b >= BIN_COUNT) b = BIN_COUNT - 1;
            return b;
        };

        for (size_t ei = 0; ei < ring_edges.size(); ++ei) {
            const Edge& e = ring_edges[ei];
            size_t b0 = clamp_to_bin(e.y_min);
            size_t b1 = clamp_to_bin(e.y_max);
            if (b0 > b1) std::swap(b0, b1);

            for (size_t b = b0; b <= b1; ++b) {
                ring_bins[b].push_back(ei);
            }
        }

        _edges.push_back(std::move(ring_edges));
        _bins.push_back(std::move(ring_bins));
        _bin_heights.push_back(bin_height);
    }
}

bool AdminArea::point_in_polygon(const Point& p) const {
    const Point projected = Point::project_mercator(p.x, p.y);
    if (projected.x < bl.x || projected.x > tr.x || projected.y < bl.y || projected.y > tr.y)
        return false;

    for (size_t r = 0; r < _edges.size(); ++r) {
        const double bin_height = _bin_heights[r];
        const auto& bins = _bins[r];
        const auto& edges = _edges[r];

        size_t bin = size_t((projected.y - bl.y) / bin_height);
        if (bin >= bins.size()) bin = bins.size() - 1;

        bool inside = false;
        const auto& bin_edges = bins[bin];

        for (size_t ei : bin_edges) {
            const Edge& e = edges[ei];

            if (projected.y >= e.y_min && projected.y < e.y_max) {
                const double x_int =
                    e.x_at_y_min + (projected.y - e.y_min) * e.inv_slope;

                if (projected.x < x_int) {
                    inside = !inside;
                }
            }
        }

        if (inside) {
            return true;
        }
    }

    return false;
}
