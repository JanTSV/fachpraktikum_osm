#include <cmath>
#include <cstddef>
#include <iostream>
#include <chrono>
#include <iomanip>

#include <optional>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <string>
#include <vector>

#include "httplib.h"

static const int PORT = 8080;

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
        Point(double x, double y) : x(x), y(y) {

        }

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

// TODO: IStringStore and IBuildingStore in here
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
            // onstruct Building and add to vector
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
        OSMHandler(ISolution& solution): _solution(solution) {

        }

        void node(const osmium::Node& node) {
            const osmium::TagList& tags = node.tags();

            if (tags.get_value_by_key("building") ||
                tags.get_value_by_key("shop") ||
                tags.get_value_by_key("amenity")) {
                if (!node.location()) {
                    return;
                }

                const char* street = tags.get_value_by_key("addr:street");
                // TODO: const char* housenumber = tags.get_value_by_key("addr:housenumber");

                // TODO: Many dont have a street name
                _solution.add_building(node.location().lat(), node.location().lon(), street);
            }
        }

        void way(const osmium::Way& way) {
            const osmium::TagList& tags = way.tags();

            if (tags.get_value_by_key("building") ||
                tags.get_value_by_key("shop") ||
                tags.get_value_by_key("amenity")) {
                if (!way.is_closed()) {
                    // std::cerr << "WARNING: Building is not a closed way." << std::endl;
                    return;
                }

                const char* street = tags.get_value_by_key("addr:street");
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
                    // TODO: Many dont have a street name (maybe a node in the list has?)
                    _solution.add_building(sum_lat / count, sum_lon / count, street);
                } else {
                    // std::cerr << "WARNING: One way consists of 0 nodes." << std::endl;
                }
            }
        }

        void relation(const osmium::Relation& rel) {
            if (!rel.tags().get_value_by_key("building")) return;

            // TODO: Relations can be buildings.
        }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: ./parser <file.osm.pbf>" << std::endl;
        return 1;
    }

    std::cout << "Parsing " << argv[1] << "..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    osmium::io::Reader reader(argv[1]);
    NaiveSolution solution;
    OSMHandler handler(solution);
    osmium::apply(reader, handler);
    reader.close();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Parsing done " << get_duration(end - start) << std::endl;
    std::cout << "NUmber of buildings: " << solution.get_buildings().size() << std::endl;

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