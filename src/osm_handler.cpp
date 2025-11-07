#include "osm_handler.hpp"

#include <cstring>
#include <iostream>
#include <charconv>

OSMHandler::OSMHandler(ISolution& solution) : _solution(solution) { }

bool OSMHandler::is_building(const osmium::TagList& tags) {
    return tags["building"];
}

bool OSMHandler::is_street(const osmium::TagList& tags) {
    return tags["highway"];
}

bool OSMHandler::is_admin_area(const osmium::TagList& tags) {
    return tags["boundary"] && std::string(tags["boundary"]) == "administrative";
}

const char* OSMHandler::get_street_name(const osmium::TagList& tags) {
    const char* street = tags["addr:street:de"];
    if (!street) {
        street = tags["addr:street"];
    }
    return street;
}

const char* OSMHandler::get_name(const osmium::TagList& tags) {
    const char* name = tags["name:de"];
    if (!name) {
        name = tags["name"];
    }
    return name;
}

std::optional<HouseNumber> OSMHandler::get_house_number(const osmium::TagList& tags) {
    std::optional<HouseNumber> house_number;
    const char* house_number_str = tags.get_value_by_key("addr:housenumber");
    if (house_number_str) {
        uint16_t n = 0;
        auto [ptr, ec] = std::from_chars(
            house_number_str,
            house_number_str + std::strlen(house_number_str),
            n
        );
        if (ec == std::errc()) {
            house_number = n;
        }
    }
    return house_number;
}

void OSMHandler::node(const osmium::Node& node) {
    const osmium::TagList& tags = node.tags();
    if (!is_building(tags)) return;
    if (!node.location()) return;

    _solution.add_building(node.location().lat(), node.location().lon(),
                           get_street_name(tags), get_house_number(tags));
}

void OSMHandler::way(const osmium::Way& way) {
    const osmium::TagList& tags = way.tags();

    if (is_building(tags)) {
        if (!way.is_closed()) return;

        double sum_lat = 0.0, sum_lon = 0.0;
        size_t count = 0;
        const auto& nodes = way.nodes();

        for (size_t i = 1; i < nodes.size(); ++i) {
            const auto& node_ref = nodes[i];
            if (node_ref.location().valid()) {
                sum_lat += node_ref.location().lat();
                sum_lon += node_ref.location().lon();
                ++count;
            }
        }

        if (count > 0) {
            _solution.add_building(sum_lat / count, sum_lon / count,
                                   get_street_name(tags), get_house_number(tags));
        }
    } else if (is_street(tags)) {
        const char* name = get_name(tags);
        if (!name) return;

        std::vector<Point> points;
        for (const auto& node_ref : way.nodes()) {
            if (node_ref.location().valid()) {
                points.emplace_back(node_ref.location().lat(), node_ref.location().lon());
            }
        }

        _solution.add_street(name, std::move(points));
    } else if (is_admin_area(tags)) {
        const char* name = get_name(tags);
        if (!name) return;

        uint8_t level = 0;
        if (tags["admin_level"]) {
            level = std::stoi(tags["admin_level"]);
        }

        std::vector<Point> points;
        for (const auto& node_ref : way.nodes()) {
            if (node_ref.location().valid()) {
                points.emplace_back(node_ref.location().lat(), node_ref.location().lon());
            }
        }

        if (!points.empty()) {
            _solution.add_admin_area(name, std::move(points), level);
        }
    }
}

void OSMHandler::area(const osmium::Area& area) {
    const osmium::TagList& tags = area.tags();

    if (is_building(tags)) {
        double sum_lat = 0.0, sum_lon = 0.0;
        size_t count = 0;
        for (const auto& nr : *area.cbegin<osmium::OuterRing>()) {
            if (nr.location().valid()) {
                sum_lat += nr.lon();
                sum_lon += nr.lat();
                ++count;
            }
        }

        if (count > 0) {
            _solution.add_building(sum_lat / count, sum_lon / count,
                                   get_street_name(tags), get_house_number(tags));
        }
    } else if (is_admin_area(tags)) {
        const char* name = get_name(tags);
        if (!name) return;

        uint8_t level = 0;
        if (tags["admin_level"]) {
            level = std::stoi(tags["admin_level"]);
        }

        std::vector<Point> points;
        for (const auto& nr : *area.cbegin<osmium::OuterRing>()) {
            if (nr.location().valid()) {
                points.emplace_back(nr.location().lat(), nr.location().lon());
            }
        }

        if (!points.empty()) {
            _solution.add_admin_area(name, std::move(points), level);
        }
    }
}
