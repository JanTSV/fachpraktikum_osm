#include "osm_handler.hpp"

#include <cstring>
#include <iostream>
#include <charconv>
#include <limits>

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

    if (is_street(tags)) {
        const char* name = get_name(tags);
        if (!name) return;

        std::vector<Point> points;
        for (const auto& node_ref : way.nodes()) {
            if (node_ref.location().valid()) {
                points.emplace_back(node_ref.location().lat(), node_ref.location().lon());
            }
        }

        _solution.add_street(name, std::move(points));
    }
}

void OSMHandler::area(const osmium::Area& area) {
    const osmium::TagList& tags = area.tags();

    if (is_building(tags)) {
        std::optional<Point> centroid = compute_centroid(area);

        if (centroid) {
            _solution.add_building((*centroid).x, (*centroid).y,
                                   get_street_name(tags), get_house_number(tags));
        }
    } else if (is_admin_area(tags)) {
        const char* name = get_name(tags);
        if (!name) return;

        uint8_t level = std::numeric_limits<uint8_t>::max();
        if (tags["admin_level"]) {
            int lvl = std::stoi(tags["admin_level"]);
            if (lvl >= std::numeric_limits<uint8_t>::min() && lvl <= std::numeric_limits<uint8_t>::max()) {
                level = static_cast<uint8_t>(lvl);
            }
            else {
                std::cout << "WARNING: LEVEL OUT OF RANGE " << lvl << std::endl;
                return;
            }
        } else {
            // Skip areas without a level.
            return;
        }
        
        std::vector<Point> points;
        for (const auto& p : *area.cbegin<osmium::OuterRing>()) {
            if (p.location().valid()) {
                points.emplace_back(p.lat(), p.lon());
            }
        }

        if (!points.empty()) {
            _solution.add_admin_area(name, std::move(points), level);
        }
    }
}

std::optional<Point> OSMHandler::compute_centroid(const osmium::Area& area) {
    double sum_lat = 0.0, sum_lon = 0.0;
    size_t count = 0;
    for (const auto& ring : area.outer_rings()) {
        for (size_t i = 0; i < ring.size(); ++i) {
            if (!ring[i].location().valid()) continue;
            sum_lat += ring[i].lat();
            sum_lon += ring[i].lon();
            ++count;
        }
    }
    if (count == 0) return std::nullopt;
   
    return Point(sum_lat / count, sum_lon / count);
}
