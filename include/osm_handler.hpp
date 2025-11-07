#pragma once

#include <optional>

#include <osmium/handler.hpp>
#include <osmium/io/any_input.hpp>

#include "isolution.hpp"
#include "geo_data.hpp"

class OSMHandler : public osmium::handler::Handler {
private:
    ISolution& _solution;

public:
    explicit OSMHandler(ISolution& solution);

    static bool is_building(const osmium::TagList& tags);
    static bool is_street(const osmium::TagList& tags);
    static bool is_admin_area(const osmium::TagList& tags);

    static const char* get_street_name(const osmium::TagList& tags);
    static const char* get_name(const osmium::TagList& tags);
    static std::optional<HouseNumber> get_house_number(const osmium::TagList& tags);

    void node(const osmium::Node& node);
    void way(const osmium::Way& way);
    void area(const osmium::Area& area);
};
