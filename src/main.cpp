#include <cmath>
#include <cstddef>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <optional>
#include <osmium/osm/tag.hpp>
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
#include "clipp.h"

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

struct Street {
    public:
        size_t name_idx;
        std::vector<Point> points;

        Street(size_t name_idx, std::vector<Point> points) : name_idx(name_idx), points(points) { }
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
        virtual void add_street(const char* name, std::vector<Point> points) = 0;
        virtual const std::vector<Building>& get_buildings() const = 0;
        virtual const std::vector<Street>& get_streets() const = 0;
};

class NaiveSolution : public ISolution {
    private:
        NaiveStringStore string_store;
        std::vector<Building> buildings;
        std::vector<Street> streets;

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

        void add_street(const char* name, std::vector<Point> points) override {
            if (!name) return;
            streets.emplace_back(string_store.get_or_add(name), points);
        }

        const std::vector<Building>& get_buildings() const override {
            return buildings;
        }

        const std::vector<Street>& get_streets() const override {
            return streets;
        }
};

class OSMHandler : public osmium::handler::Handler {
    private:
        ISolution& _solution;

    public:
        OSMHandler(ISolution& solution): _solution(solution) { }

        static bool is_building(const osmium::TagList& tags) {
            return tags["building"];
        }

        static bool is_street(const osmium::TagList& tags) {
            return tags["highway"];
        }

        static const char* get_street_name(const osmium::TagList& tags) {
            // Check for german spelling first
            const char* street = tags["addr:street:de"];
            if (!street) {
                // Get native spelling otherwise
                street = tags["addr:street"];
            }

            return street;
        }

        static const char* get_name(const osmium::TagList& tags) {
            // Check for german spelling first
            const char* street = tags["name:de"];
            if (!street) {
                // Get native spelling otherwise
                street = tags["name"];
            }

            return street;
        }

        void node(const osmium::Node& node) {
            const osmium::TagList& tags = node.tags();

            // Nodes are just buildings for now.
            if (!is_building(tags)) return;

            if (!node.location()) return;

            // TODO: const char* housenumber = tags.get_value_by_key("addr:housenumber");

            _solution.add_building(node.location().lat(), node.location().lon(), get_street_name(tags));
        }

        void way(const osmium::Way& way) {
            const osmium::TagList& tags = way.tags();

            if (is_building(tags)) {
                if (!way.is_closed()) return;

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
            } else if (is_street(tags)) {
                // Streets have to have a name
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

        void area(const osmium::Area& area) {
            const osmium::TagList& tags = area.tags();

            if (!is_building(tags)) return;

            // TODO: const char* housenumber = tags.get_value_by_key("addr:housenumber");

            // Compute centroid
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
                _solution.add_building(sum_lat / count, sum_lon / count, get_street_name(tags));
            } else {
                // std::cerr << "WARNING: One way consists of 0 nodes." << std::endl;
            }
        }
};

struct Configuration {
    public:
        osmium::io::File input_file;
        bool in_binary_file = false;
        std::optional<std::string> out_binary_file;

};

int main(int argc, char* argv[]) {
    Configuration configuration;

    // Command line parsing
    auto cli = clipp::group(
        clipp::value("input file", [&configuration](const std::string& path) {
            configuration.input_file = osmium::io::File(path);
            return true;
        }).doc("Path to OSM file"),

        clipp::option("-bi", "--binary_in").set(configuration.in_binary_file).doc("Input file is in binary format"),
        

        clipp::option("-bo", "--binary_out").call([&configuration](const std::string& s) {
            configuration.out_binary_file = s;
        }).doc("Optional output binary file")
    );

    if (!clipp::parse(argc, argv, cli)) {
        std::cout << clipp::make_man_page(cli, argv[0]);
        return 1;
    }

    std::cout << "Parsing " << argv[1] << "..." << std::endl;
    auto start_parsing = std::chrono::high_resolution_clock::now();

    // Create areas out of multipolygons
    osmium::area::Assembler::config_type assembler_config;
    assembler_config.create_empty_areas = false;

    osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{assembler_config};
    osmium::relations::read_relations(configuration.input_file, mp_manager);

    index_type index;
    location_handler_type location_handler{index};
    location_handler.ignore_errors();

    // Actual parsing of constructed areas and nodes
    NaiveSolution solution;
    OSMHandler handler(solution);
    osmium::io::Reader reader{configuration.input_file, osmium::io::read_meta::no};
    osmium::apply(reader, location_handler, handler, mp_manager.handler([&handler](const osmium::memory::Buffer& area_buffer) {
        osmium::apply(area_buffer, handler);
    }));
    reader.close();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Parsing done " << get_duration(end - start_parsing) << std::endl;
    std::cout << "Number of buildings: " << solution.get_buildings().size() << std::endl;
    std::cout << "Number of streets: " << solution.get_streets().size() << std::endl;

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