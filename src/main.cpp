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

// Here we only have a 2DTree
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
            return "[]";
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

class NaiveSolution : public ISolution {
    public:
        void add_building(double lat, double lon, const char* street, std::optional<HouseNumber> house_number) override {
            // Store street name, it building has one tagged
            std::optional<size_t> street_idx;
            if (street) {
                street_idx = _string_store.get_or_add(street);
            }
            // Construct building and add to vector
            Point location{lat, lon};
            _buildings.emplace_back(location, street_idx, house_number);
        }

        void add_street(const char* name, std::vector<Point> points) override {
            if (!name) return;
            _streets.emplace_back(_string_store.get_or_add(name), points);
        }

        void add_admin_area(const char* name, std::vector<Point> boundary, uint8_t level) override {
            if (!name) return;
            _admin_areas.emplace_back(_string_store.get_or_add(name), boundary, level);
        }

        size_t num_buildings() const override {
            return _buildings.size();
        }

        size_t num_streets() const override {
            return _streets.size();
        }

        size_t num_admin_areas() const override {
            return _admin_areas.size();
        }

        std::string get_buildings_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) override {
            if (_get_buildings_in_view_first_call) {
                _sent_buildings.resize((num_buildings() + 63) / 64, 0);
                _get_buildings_in_view_first_call = false;
            }

            std::ostringstream json;
            json << "[";
            bool first = true;

            for (size_t i = 0; i < _buildings.size(); ++i) {
                if ((_sent_buildings[i / 64] & (1 << (i % 64))) != 0) continue;
                
                const auto& b = _buildings[i];
                if (b.location.x >= sw_lat && b.location.x <= ne_lat &&
                    b.location.y >= sw_lon && b.location.y <= ne_lon) {
                    if (!first) json << ",";
                    json << "[" << b.location.x << "," << b.location.y << "]";
                    _sent_buildings[i / 64] |= 1U << (i % 64);
                    first = false;
                }
            }
            json << "]";

            return json.str();
        }

        void serialize(const std::string& path) const override {
            std::ofstream ofs(path, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << *this;
        }

        void preprocess() override {
            // Find streets of buildings
            for (auto& building : _buildings) {
                if (!building.street_idx) {
                    Street& nearest_street = find_nearest_street(building);
                    building.street_idx = nearest_street.name_idx;
                }
            }

            // Housenumber interpolation (undoable in good time here)
        }

    private:
        NaiveStringStore _string_store;
        std::vector<Building> _buildings;
        std::vector<Street> _streets;
        std::vector<AdminArea> _admin_areas;
        std::vector<uint64_t> _sent_buildings;
        bool _get_buildings_in_view_first_call = true;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _string_store;
            ar & _buildings;
            ar & _streets;
            ar & _admin_areas;
        }

        Street& find_nearest_street(Building& building) {
            Street& nearest = _streets[0];
            double min_dist = std::numeric_limits<double>::max();

            for (auto& street : _streets) {
                double min_dist_street = std::numeric_limits<double>::max();
                for (auto& point : street.points) {
                    min_dist_street = std::min<double>(min_dist_street, building.location.euclidean_distance(point));
                }

                if (min_dist_street < min_dist) {
                    min_dist = min_dist_street;
                    nearest = street;
                }
            }

            return nearest;
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

    std::cout << "Server started at http://localhost:" << configuration.port << std::endl;
    svr.listen("localhost", configuration.port);

    return 0;
}