#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <optional>
#include <string>
#include <fstream>

#include <osmium/io/any_input.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/osm/tag.hpp>
#include <osmium/visitor.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <vector>

#include "httplib.h"
#include "clipp.h"
#include "geo_data.hpp"
#include "isolution.hpp"
#include "string_store.hpp"
#include "osm_handler.hpp"

static const int DEFAULT_PORT = 8080;
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

template<typename Duration>
std::string get_duration(const Duration& duration) {
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

    std::ostringstream out;
    out << std::fixed << std::setprecision(2);

    out << "[";
    if (us < 1000) {
        out << us << "µs";
    } else if (us < 1000000) {
        out << us / 1000.0 << "ms";
    } else {
        out << us / 1000000.0 << "s";
    }
    out << "]";

    return out.str();
}

// Here we only need a 2DTree
class KDTree {
    public:
        KDTree() : _root(nullptr), _size(0) { }

        void insert(const Point& point, size_t idx) {
            _size++;

            if (!_root) {
                _root = std::make_unique<Node>(point, idx);
                return;
            }

            Node* current = _root.get();
            size_t depth = 0;

            while (true) {
                size_t cd = depth % 2;  // 2 dims

                if (point[cd] < current->point[cd]) {
                    if (current->left) {
                        current = current->left.get();
                    } else {
                        current->left = std::make_unique<Node>(point, idx);
                        break;
                    }
                } else {
                    if (current->right) {
                        current = current->right.get();
                    } else {
                        current->right = std::make_unique<Node>(point, idx);
                        break;
                    }
                }

                ++ depth;
            }
        }

        std::vector<std::pair<size_t, std::array<double, 2>>> range_search(double min_lat, double min_lon, double max_lat, double max_lon) const {
            std::vector<std::pair<size_t, std::array<double, 2>>> result;
            if (!_root) return result;

            std::vector<std::pair<const Node*, size_t>> stack;
            stack.emplace_back(_root.get(), 0);

            while (!stack.empty()) {
                const auto [node, depth] = stack.back();
                stack.pop_back();
                if (!node) continue;

                const double lat = node->point.x;
                const double lon = node->point.y;

                // check if point is inside bounding box
                if (lat >= min_lat && lat <= max_lat &&
                    lon >= min_lon && lon <= max_lon) {
                    const size_t idx = node->idx;
                    result.emplace_back(node->idx, std::array<double, 2>{lat, lon});
                }

                const size_t cd = depth % 2;
                const double coord = (cd == 0) ? lat : lon;
                const double min_bound = (cd == 0) ? min_lat : min_lon;
                const double max_bound = (cd == 0) ? max_lat : max_lon;

                if (min_bound <= coord && node->left)
                    stack.emplace_back(node->left.get(), depth + 1);
                if (max_bound >= coord && node->right)
                    stack.emplace_back(node->right.get(), depth + 1);
            }

            return result;
        }

        std::optional<std::pair<size_t, std::array<double, 2>>> find_nearest(const Point& target) {
            if (!_root) return std::nullopt;

            const Node* best = nullptr;
            double best_dist = std::numeric_limits<double>::max();

            // node, depth
            std::vector<std::pair<const Node*, size_t>> stack;
            stack.emplace_back(_root.get(), 0);

            while (!stack.empty()) {
                auto [node, depth] = stack.back();
                stack.pop_back();
                if (!node) continue;

                const double dist = target.euclidean_distance(node->point);
                if (dist < best_dist) {
                    best_dist = dist;
                    best = node;
                }

                size_t cd = depth % 2;
                const Node* near = nullptr;
                const Node* far = nullptr;

                if (target[cd] < node->point[cd]) {
                    near = node->left.get();
                    far = node->right.get();
                } else {
                    near = node->right.get();
                    far = node->left.get();
                }

                stack.emplace_back(near, depth + 1);

                if (far && std::abs(target[cd] - node->point[cd]) < best_dist) {
                    stack.emplace_back(far, depth + 1);
                }
            }

            if (best) return std::make_pair(best->idx, std::array<double, 2>{best->point.x, best->point.y});
            return std::nullopt;
        }

        size_t size() const {
            return _size;
        }

    private:
        struct Node {
            public:
                Point point;
                size_t idx;
                std::unique_ptr<Node> left;
                std::unique_ptr<Node> right;

                Node() = default;
                Node(const Point point, size_t idx) : point(point), idx(idx), left(nullptr), right(nullptr) { }

            private:
                friend class boost::serialization::access;
                template<class Archive>
                void serialize(Archive& ar, const unsigned int /*version*/) {
                    ar & point;
                    ar & idx;
                    ar & left;
                    ar & right;
                }
        };

        std::unique_ptr<Node> _root;
        size_t _size;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _root;
            ar & _size;
        }
};

class KDSolution : public ISolution {
    public:
        void add_building(double lat, double lon, const char* street, std::optional<HouseNumber> house_number) override {
            // Add meta info to _buildings
            std::optional<size_t> street_idx;
            if (street) {
                street_idx = _string_store.get_or_add(street);
            }
            _buildings.emplace_back(street_idx, house_number);

            // Add building do 2d tree
            Point location(lat, lon);
            _buildings_tree.insert(location, _buildings.size() - 1);
        }

        void add_street(const char* name, std::vector<Point> points) override {
            // Skip streets without names
            if (!name) return;

            // Add meta info to _streets
            _streets.emplace_back(_string_store.get_or_add(name));
            const size_t street_idx = _streets.size() - 1;

            // Add all street points to 2d tree
            for (auto point : points) {
                _streets_tree.insert(point, street_idx);
            }
        }

        void add_admin_area(const char* name, std::vector<Point> boundary, uint8_t level) override {

        }

        size_t num_buildings() const override {
            return _buildings_tree.size();
        }

        size_t num_streets() const override {
            return _streets_tree.size();
        }

        size_t num_admin_areas() const override {
            return 0;
        }

        std::string get_buildings_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) override {
            std::ostringstream json;
            json << "[";
            bool first = true;

            std::vector<std::pair<size_t, std::array<double, 2>>> buildings = _buildings_tree.range_search(sw_lat, sw_lon, ne_lat, ne_lon);
            for (auto &[idx, coords] : buildings) {
                if (!first) json << ",";
                json << "[" << coords[0] << "," << coords[1] << "]";
                first = false;
            }
            json << "]";

            return json.str();
        }

        std::string get_nearest_building(double lat, double lon) {
            Point target(lat, lon);
            auto nearest_building = _buildings_tree.find_nearest(target);
            if (!nearest_building) return "[]";

            auto [idx, coords] = *nearest_building;
            std::ostringstream json;
            json << "{";
            json << "\"lat\":" << coords[0] << ",";
            json << "\"lon\":" << coords[1] << ",";

            if (_buildings[idx].street_idx) {
                json << "\"street\":\"" << _string_store.get(*_buildings[idx].street_idx) << "\",";
            } else {
                json << "\"street\":null,";
            }

            if (_buildings[idx].house_number) {
                json << "\"house_number\":\"" << *_buildings[idx].house_number << "\"";
            } else {
                json << "\"house_number\":null";
            }

            json << "}";
            return json.str();
        }

        void preprocess() override {

        }

        void serialize(const std::string& path) const override {
            std::ofstream ofs(path, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << *this;
        }

    private:
        struct BuildingData {
            public:
                std::optional<size_t> street_idx;
                std::optional<HouseNumber> house_number;

                BuildingData() = default;
                BuildingData(std::optional<size_t> street_idx, std::optional<HouseNumber> house_number) : street_idx(street_idx), house_number(house_number) { }

            private:
                friend class boost::serialization::access;
                template<class Archive>
                void serialize(Archive& ar, const unsigned int /*version*/) {
                    ar & street_idx;
                    ar & house_number;
                }
        };

        MappedStringStore _string_store;

        KDTree _buildings_tree;
        std::vector<BuildingData> _buildings;

        KDTree _streets_tree;
        std::vector<size_t> _streets;  // Name idx

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _string_store;
            ar & _buildings_tree;
            ar & _buildings;
            ar & _streets_tree;
            ar & _streets;
        }

};

struct Configuration {
    public:
        osmium::io::File input_file;
        bool in_binary_file = false;
        bool help = false;
        std::optional<std::string> out_binary_file;
        int port = DEFAULT_PORT;
};

int main(int argc, char* argv[]) {
    Configuration configuration;

    // Command line parsing
    auto cli = (
        clipp::option("-h", "--help").set(configuration.help).doc("Print this help"),

        clipp::option("-bi", "--binary_in")
            .set(configuration.in_binary_file)
            .doc("Input file is in binary format"),

        (clipp::option("-bo", "--binary_out") &
        clipp::value("output file", [&configuration](const std::string& path) {
                configuration.out_binary_file = path;
                return true;
            })).doc("Optional path to output binary file"),

        (clipp::option("-p", "--port") & clipp::value("port", configuration.port)).doc(std::string("Port number for the localhost. DEFAULT = ") + std::to_string(DEFAULT_PORT)),

        clipp::value("input file", [&configuration](const std::string& path) {
            configuration.input_file = osmium::io::File(path);
            return true;
        }).doc("Path to OSM file")
    );

    if (!clipp::parse(argc, argv, cli, true) || configuration.help) {
        std::cout << clipp::make_man_page(cli, argv[0]);
        return 1;
    }

    KDSolution solution;
    if (configuration.in_binary_file) {
        std::cout << "Loading binary data from " << configuration.input_file.filename() << "..." << std::endl;
        auto start_load = std::chrono::high_resolution_clock::now();

        std::ifstream ifs(configuration.input_file.filename(), std::ios::binary);
        if (!ifs.is_open()) {
            std::cerr << "Error: Cannot open binary file " << configuration.input_file.filename() << std::endl;
            return 1;
        }

        boost::archive::binary_iarchive ia(ifs);
        ia >> solution;

        auto end_load = std::chrono::high_resolution_clock::now();
        std::cout << "Loaded binary data " << get_duration(end_load - start_load) << std::endl;
    } else {
        std::cout << "Parsing " << configuration.input_file.filename() << "..." << std::endl;
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
        OSMHandler handler(solution);
        osmium::io::Reader reader{configuration.input_file, osmium::io::read_meta::no};
        osmium::apply(reader, location_handler, handler, mp_manager.handler([&handler](const osmium::memory::Buffer& area_buffer) {
            osmium::apply(area_buffer, handler);
        }));
        reader.close();
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "Parsing done " << get_duration(end - start_parsing) << std::endl;
    }
   
    std::cout << "Number of buildings: " << solution.num_buildings() << std::endl;
    std::cout << "Number of streets: " << solution.num_streets() << std::endl;
    std::cout << "Number of admin areas: " << solution.num_admin_areas() << std::endl;

    // Serialize solution if argument is set
    if (configuration.out_binary_file) {
        std::cout << "Serializing to " << *configuration.out_binary_file << "..." << std::endl;
        auto start_ser = std::chrono::high_resolution_clock::now();

        solution.serialize(*configuration.out_binary_file);

        auto end_ser = std::chrono::high_resolution_clock::now();
        std::cout << "Serialization done " << get_duration(end_ser - start_ser) << std::endl;
    }

    // Preprocessing (maybe before serializing)
    // std::cout << "Preprocessing..." << std::endl;
    // auto start_prep = std::chrono::high_resolution_clock::now();

    // solution.preprocess();

    // auto end_prep = std::chrono::high_resolution_clock::now();
    // std::cout << "Preprocessing done " << get_duration(end_prep - start_prep) << std::endl;

    httplib::Server svr;
    svr.set_mount_point("/", "./www");

    // API endpoint for buildings
    svr.Post("/buildings", [&solution](const httplib::Request &req, httplib::Response &res) {
        // Read viewport boundary
        double sw_lat, sw_lon, ne_lat, ne_lon;
        bool ok = (sscanf(
            req.body.c_str(),
            R"({"swLat":%lf,"swLon":%lf,"neLat":%lf,"neLon":%lf})",
            &sw_lat, &sw_lon, &ne_lat, &ne_lon
        ) == 4);

        if (!ok) {
            res.status = 400;
            res.set_content("Invalid JSON format", "text/plain");
            return;
        }

        res.status = 200;
        res.set_content(solution.get_buildings_in_view(sw_lat, sw_lon, ne_lat, ne_lon), "application/json");
    });

    // API endpoint for reverse geocoder
    svr.Post("/nearest", [&solution](const httplib::Request &req, httplib::Response &res) {
        double lat, lon;
        bool ok = (sscanf(req.body.c_str(), R"({"lat":%lf,"lon":%lf})", &lat, &lon) == 2);
        if (!ok) {
            res.status = 400;
            res.set_content("Invalid JSON format", "text/plain");
            return;
        }
        res.status = 200;
        res.set_content(solution.get_nearest_building(lat, lon), "application/json");
    });

    std::cout << "Server started at http://localhost:" << configuration.port << std::endl;
    svr.listen("localhost", configuration.port);

    return 0;
}