#pragma once

#include "geo_data.hpp"

class ISolution {
    public:
        virtual ~ISolution() {}
        virtual void add_building(double lat, double lon, const char* street, std::optional<HouseNumber> house_number) = 0;
        virtual void add_street(const char* name, std::vector<Point> points) = 0;
        virtual void add_admin_area(const char* name, std::vector<Point> boundary, uint8_t level) = 0;
        virtual size_t num_buildings() const = 0;
        virtual size_t num_streets() const = 0;
        virtual size_t num_admin_areas() const = 0;
        virtual std::string get_buildings_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) const = 0;
        virtual std::string get_streets_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) const = 0;
        virtual std::string get_nearest_building(double lat, double lon) const = 0;
        virtual void serialize(const std::string& path) const = 0;
        virtual void preprocess() = 0;
};
