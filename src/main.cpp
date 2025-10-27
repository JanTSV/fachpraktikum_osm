#include <cmath>
#include <cstddef>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <optional>
#include <string>
#include <vector>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/visitor.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>

#include "httplib.h"

static const int PORT = 8080;
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

template<typename Duration>
std::string get_duration(const Duration& duration) {
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

    std::ostringstream out;
    out << std::fixed << std::setprecision(2);

    out << "[";
    if (us < 1000) {
        out << us << " µs";
    } else if (us < 1000000) {
        out << us / 1000.0 << " ms";
    } else {
        out << us / 1000000.0 << " s";
    }
    out << "]";

    return out.str();
}

struct Point {
    public:
        double x;  // lat
        double y;  // lon
        Point(double x, double y) : x(x), y(y) { }

        double euclidean_distance(Point& other) {
            return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
        }
};

struct Building {
    public:
        Point location;
        std::optional<size_t> street_idx;
        // TODO: house number

        Building(Point location, std::optional<size_t> street_idx) : location(location), street_idx(street_idx) { }
};

struct NaiveStringStore {
    private:
        std::vector<std::string> _data;

    public:
        size_t get_or_add(std::string value) {
            // Very naive string search in O(m*n)
            for (size_t i = 0; i < _data.size(); i++) {
                if (value == _data[i]) {
                    return i;
                }
            }

            _data.push_back(value);
            return _data.size() - 1;
        }
};

class ISolution {
    public:
        virtual ~ISolution() {}
        virtual void add_building(double lat, double lon, const char* street) = 0;
        virtual const std::vector<Building>& get_buildings() const = 0;
};

class NaiveSolution : public ISolution {
    private:
        NaiveStringStore string_store;
        std::vector<Building> buildings;

    public:
        void add_building(double lat, double lon, const char* street) override {
            // Store street name, it building has one tagged
            std::optional<size_t> street_idx;
            if (street) {
                street_idx = string_store.get_or_add(street);
            }
            // Construct building and add to vector
            Point location{lat, lon};
            buildings.emplace_back(location, street_idx);
        }


        const std::vector<Building>& get_buildings() const override {
            return buildings;
        }
};

class OSMHandler : public osmium::handler::Handler {
    private:
        ISolution& _solution;

    public:
        OSMHandler(ISolution& solution): _solution(solution) { }

        bool is_building(const osmium::TagList& tags) {
            return tags["building"] || tags["shop"];
        }

        const char* get_street_name(const osmium::TagList& tags) {
            // Check for german spelling first
            const char* street = tags.get_value_by_key("addr:street:de");
            if (!street) {
                // Get native spelling otherwise
                street = tags.get_value_by_key("addr:street");
            }

            return street;
        }

        void node(const osmium::Node& node) {
            const osmium::TagList& tags = node.tags();

            if (!is_building(tags)) return;

            if (!node.location()) {
                return;
            }

            // TODO: const char* housenumber = tags.get_value_by_key("addr:housenumber");

            _solution.add_building(node.location().lat(), node.location().lon(), get_street_name(tags));
        }

        void way(const osmium::Way& way) {
            const osmium::TagList& tags = way.tags();

            if (!is_building(tags)) return;

            if (!way.is_closed()) {
                // std::cerr << "WARNING: Building is not a closed way." << std::endl;
                return;
            }

            // TODO: const char* housenumber = tags.get_value_by_key("addr:housenumber");
            
            // Compute centroid
            double sum_lat = 0.0, sum_lon = 0.0;
            size_t count = 0;
            const auto& nodes = way.nodes();

            // Can skip first node since its a closed way
            for (size_t i = 1; i < nodes.size(); ++i) {
                const auto& node_ref = nodes[i];
                if (node_ref.location().valid()) {
                    sum_lat += node_ref.location().lat();
                    sum_lon += node_ref.location().lon();
                    ++count;
                }
            }

            if (count > 0) {
                _solution.add_building(sum_lat / count, sum_lon / count, get_street_name(tags));
            } else {
                // std::cerr << "WARNING: One way consists of 0 nodes." << std::endl;
            }
        }

        void area(const osmium::Area& area) {
            const osmium::TagList& tags = area.tags();

            if (!is_building(tags)) return;

            // TODO: const char* housenumber = tags.get_value_by_key("addr:housenumber");

            // Compute centroid
            double sum_lat = 0.0, sum_lon = 0.0;
            size_t count = 0;
            for (const auto& nr : *area.cbegin<osmium::OuterRing>()) {
                sum_lat += nr.lon();
                sum_lon += nr.lat();
                ++count;
            }

            if (count > 0) {
                _solution.add_building(sum_lat / count, sum_lon / count, get_street_name(tags));
            } else {
                // std::cerr << "WARNING: One way consists of 0 nodes." << std::endl;
            }
        }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: ./parser <file.osm.pbf>" << std::endl;
        return 1;
    }

    std::cout << "Parsing " << argv[1] << "..." << std::endl;
    auto start_parsing = std::chrono::high_resolution_clock::now();

    // Create areas out of multipolygons
    const osmium::io::File input_file{argv[1]};
    osmium::area::Assembler::config_type assembler_config;
    assembler_config.create_empty_areas = false;

    osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{assembler_config};
    osmium::relations::read_relations(input_file, mp_manager);

    index_type index;
    location_handler_type location_handler{index};
    location_handler.ignore_errors();

    // Actual parsing of constructed areas and nodes
    NaiveSolution solution;
    OSMHandler handler(solution);
    osmium::io::Reader reader{input_file, osmium::io::read_meta::no};
    osmium::apply(reader, location_handler, handler, mp_manager.handler([&handler](const osmium::memory::Buffer& area_buffer) {
        osmium::apply(area_buffer, handler);
    }));
    reader.close();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Parsing done " << get_duration(end - start_parsing) << std::endl;
    std::cout << "Number of buildings: " << solution.get_buildings().size() << std::endl;

    httplib::Server svr;
    svr.set_mount_point("/", "./www");

    // API endpoint for cities
    svr.Get("/buildings", [&solution](const httplib::Request&, httplib::Response& res) {
        const auto& buildings = solution.get_buildings();
        std::ostringstream json;

        json << "[";
        for (size_t i = 0; i < buildings.size(); ++i) {
            const auto& b = buildings[i];
            json << "[" << b.location.x << "," << b.location.y << "]";
            if (i + 1 < buildings.size()) json << ",";
        }
        json << "]";

        res.set_content(json.str(), "application/json");
    });

    std::cout << "Server started at http://localhost:" << PORT << std::endl;
    svr.listen("localhost", PORT);

    return 0;
}