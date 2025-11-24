#pragma once

#include <optional>
#include <vector>
#include <cstdint>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/access.hpp>

using HouseNumber = uint16_t;

static constexpr double R = 6378137.0;

struct Point {
public:
    double x;
    double y;

    Point();
    Point(double x, double y);

    double& operator[](size_t idx);
    const double& operator[](size_t idx) const;

    double euclidean_distance(const Point& other) const;

    static Point project_mercator(double lat, double lon);

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & x;
        ar & y; 
    }
};

struct Building {
public:
    Point location;
    std::optional<size_t> street_idx;
    std::optional<HouseNumber> house_number;
    size_t address;

    Building();
    Building(Point location, std::optional<size_t> street_idx, std::optional<HouseNumber> house_number);

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & location;
        ar & street_idx;
        ar & house_number;
        ar & address;
    }
};

struct Street {
public:
    size_t name_idx;
    std::vector<Point> points;

    Street();
    Street(size_t name_idx, std::vector<Point> points);

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & name_idx;
        ar & points;
    }
};

struct AdminArea {
public:
    size_t name_idx;
    std::vector<Point> boundary;
    Point bl;  // Bounding box
    Point tr;
    uint8_t level;

    AdminArea();
    AdminArea(size_t name_idx, std::vector<Point> boundary, uint8_t level);

    bool point_in_polygon(const Point& p) const;

private:

    std::vector<Point> _projected_boundary;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & name_idx;
        ar & boundary;
        ar & level;
        ar & bl;
        ar & tr;
        ar & _projected_boundary;
    }
};
