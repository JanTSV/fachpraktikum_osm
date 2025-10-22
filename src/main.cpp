#include <cmath>
#include <cstddef>
#include <iostream>
#include <chrono>
#include <iomanip>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <string>

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
    private:
        double _x;  // lat
        double _y;  // lon

    public:
        Point(double x, double y) : _x(x), _y(y) {

        }

        double euclidean_distance(Point& other) {
            return std::sqrt(std::pow(_x - other._x, 2) + std::pow(_y - other._y, 2));
        }
};

struct Building {
    private:
        Point _location;
        size_t _street_idx;
        // TODO: house number

    public:
        Building(Point location, size_t street_idx) : _location(location), _street_idx(street_idx) {

        }
};

class ISolution {
    public:
        virtual ~ISolution() {}
        virtual void add_building(Building building) = 0;
};

class NaiveSolution : public ISolution {
    public:
        void add_building(Building building) override {

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

                std::cout << "Node ID: " << node.id() << std::endl;
            }
        }

        void way(const osmium::Way& way) {
            const osmium::TagList& tags = way.tags();

            if (tags.get_value_by_key("building") ||
                tags.get_value_by_key("shop") ||
                tags.get_value_by_key("amenity")) {
                if (!way.is_closed()) {
                    return;
                }

                std::cout << "Way ID: " << way.id() << std::endl;
            }
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

    httplib::Server svr;
    svr.set_mount_point("/", "./www");

    // TODO: API endpoint for cities
    svr.Get("/points", [](const httplib::Request&, httplib::Response& res) {
    });

    std::cout << "Server started at http://localhost:" << PORT << std::endl;
    svr.listen("localhost", PORT);

    return 0;
}