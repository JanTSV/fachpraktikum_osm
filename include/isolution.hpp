#pragma once

#include "geo_data.hpp"

class ISolution {
    public:
        virtual ~ISolution() {}
        virtual void add_building(double lat, double lon, const char* street, std::optional<HouseNumber> house_number) = 0;
        virtual void add_street(const char* name, std::vector<Point> points) = 0;
        virtual void add_admin_area(const char* name, std::vector<Point> boundary, uint8_t level) = 0;
        virtual const std::vector<Building>& get_buildings() const = 0;
        virtual const std::vector<Street>& get_streets() const = 0;
        virtual const std::vector<AdminArea>& get_admin_areas() const = 0;
        virtual void serialize(const std::string& path) const = 0;
        virtual void preprocess() = 0;
};
