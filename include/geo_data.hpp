#pragma once

#include <optional>
#include <vector>
#include <cstdint>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/access.hpp>

using HouseNumber = uint16_t;

struct Point {
public:
    double x;
    double y;

    Point();
    Point(double x, double y);

    double& operator[](size_t idx);
    const double& operator[](size_t idx) const;

    double euclidean_distance(const Point& other) const;

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

    Building();
    Building(Point location, std::optional<size_t> street_idx, std::optional<HouseNumber> house_number);

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & location;
        ar & street_idx;
        ar & house_number;
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
    uint8_t level;

    AdminArea();
    AdminArea(size_t name_idx, std::vector<Point> boundary, uint8_t level);

    bool point_in_polygon(const Point& p) const;

private:
    // Bounding box
    Point _bl;
    Point _tr;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & name_idx;
        ar & boundary;
        ar & level;
        ar & _bl;
        ar & _tr;
    }
};
