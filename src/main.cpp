#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <optional>
#include <osmium/osm/tag.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <charconv>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/osm/area.hpp>
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

#include "httplib.h"
#include "clipp.h"

static const int PORT = 8080;
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;
using HouseNumber = uint16_t;

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

struct Point {
    public:
        double x;  // lat
        double y;  // lon

        Point() : x(0), y(0) {}
        Point(double x, double y) : x(x), y(y) { }

        double euclidean_distance(Point& other) {
            return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
        }

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & x;
            ar & y;
        }
};

struct Building {
    public:
        Point location;
        std::optional<size_t> street_idx;
        std::optional<HouseNumber> house_number;
        // TODO: house number

        Building() : location(), street_idx(std::nullopt), house_number(std::nullopt) { }
        Building(Point location, std::optional<size_t> street_idx, std::optional<HouseNumber> house_number) : location(location), street_idx(street_idx), house_number(house_number) { }

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & location;
            ar & street_idx;
            ar & house_number;
        }
};

struct Street {
    public:
        size_t name_idx;
        std::vector<Point> points;
        
        Street() : name_idx(0), points() { }
        Street(size_t name_idx, std::vector<Point> points) : name_idx(name_idx), points(points) { }
    
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & name_idx;
            ar & points;
        }
};

struct AdminArea {
    public:
        size_t name_idx;
        std::vector<Point> boundary;
        uint8_t level;

        AdminArea() : name_idx(0), boundary(), level(0) { }
        AdminArea(size_t name_idx, std::vector<Point> boundary, uint8_t level) : name_idx(name_idx), boundary(boundary), level(level) { }

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & name_idx;
            ar & boundary;
            ar & level;
        }
};

struct NaiveStringStore {
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

    private:
        std::vector<std::string> _data;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _data;
        }
};

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

        const std::vector<Building>& get_buildings() const override {
            return _buildings;
        }

        const std::vector<Street>& get_streets() const override {
            return _streets;
        }

        const std::vector<AdminArea>& get_admin_areas() const override {
            return _admin_areas;
        }

        void serialize(const std::string& path) const override {
            std::ofstream ofs(path, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << *this;
        }

    private:
        NaiveStringStore _string_store;
        std::vector<Building> _buildings;
        std::vector<Street> _streets;
        std::vector<AdminArea> _admin_areas;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _string_store;
            ar & _buildings;
            ar & _streets;
            ar & _admin_areas;
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

        static bool is_admin_area(const osmium::TagList& tags) {
            return tags["boundary"] && std::string(tags["boundary"]) == "administrative";
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


        static std::optional<HouseNumber> get_house_number(const osmium::TagList& tags) {
            std::optional<HouseNumber> house_number;

            const char* house_number_str = tags.get_value_by_key("addr:housenumber");
            if (house_number_str) {
                // std::cout << "House number: " << house_number_str << std::endl;
                uint16_t n = 0;
                auto [ptr, ec] = std::from_chars(
                    house_number_str,
                    house_number_str + std::strlen(house_number_str),
                    n
                );

                if (ec == std::errc()) {
                    house_number = n;
                } else {
                    // std::cerr << "WARNING: invalid or out-of-range house number: " << house_number_str << std::endl;
                }
            }

            return house_number;
        }

        void node(const osmium::Node& node) {
            const osmium::TagList& tags = node.tags();

            // Nodes are just buildings for now.
            if (!is_building(tags)) return;

            if (!node.location()) return;

            _solution.add_building(node.location().lat(), node.location().lon(), get_street_name(tags), get_house_number(tags));
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
                    _solution.add_building(sum_lat / count, sum_lon / count, get_street_name(tags), get_house_number(tags));
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
            } else if (is_admin_area(tags)) {
                // AdminAreas have to have a name
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

        void area(const osmium::Area& area) {
            const osmium::TagList& tags = area.tags();

            if (is_building(tags)) {
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
                    _solution.add_building(sum_lat / count, sum_lon / count, get_street_name(tags), get_house_number(tags));
                } else {
                    // std::cerr << "WARNING: One way consists of 0 nodes." << std::endl;
                }
            } else if (is_admin_area(tags)) {
                // AdminAreas have to have a name
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
};

struct Configuration {
    public:
        osmium::io::File input_file;
        bool in_binary_file = false;
        bool help = false;
        std::optional<std::string> out_binary_file;

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

        clipp::value("input file", [&configuration](const std::string& path) {
            configuration.input_file = osmium::io::File(path);
            return true;
        }).doc("Path to OSM file")
    );

    if (!clipp::parse(argc, argv, cli, true) || configuration.help) {
        std::cout << clipp::make_man_page(cli, argv[0]);
        return 1;
    }

    NaiveSolution solution;
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
   
    std::cout << "Number of buildings: " << solution.get_buildings().size() << std::endl;
    std::cout << "Number of streets: " << solution.get_streets().size() << std::endl;
    std::cout << "Number of admin areas: " << solution.get_admin_areas().size() << std::endl;

    // Serialize solution if argument is set
    if (configuration.out_binary_file) {
        std::cout << "Serializing to " << *configuration.out_binary_file << "..." << std::endl;
        auto start_ser = std::chrono::high_resolution_clock::now();

        solution.serialize(*configuration.out_binary_file);

        auto end_ser = std::chrono::high_resolution_clock::now();
        std::cout << "Serialization done " << get_duration(end_ser - start_ser) << std::endl;
    }

    httplib::Server svr;
    svr.set_mount_point("/", "./www");

    std::vector<uint64_t> sent_buildings((solution.get_buildings().size() + 63) / 64, 0);

    // API endpoint for buildings
    svr.Post("/buildings", [&solution, &sent_buildings](const httplib::Request &req, httplib::Response &res) {
        // Read viewport boundary
        double swLat, swLon, neLat, neLon;
        bool ok = (sscanf(
            req.body.c_str(),
            R"({"swLat":%lf,"swLon":%lf,"neLat":%lf,"neLon":%lf})",
            &swLat, &swLon, &neLat, &neLon
        ) == 4);

        if (!ok) {
            res.status = 400;
            res.set_content("Invalid JSON format", "text/plain");
            return;
        }

        const auto& buildings = solution.get_buildings();

        std::ostringstream json;
        json << "[";
        bool first = true;
        for (size_t i = 0; i < buildings.size(); ++i) {
            if ((sent_buildings[i / 64] & (1 << (i % 64))) != 0) continue;
            
            const auto& b = buildings[i];
            if (b.location.x >= swLat && b.location.x <= neLat &&
                b.location.y >= swLon && b.location.y <= neLon) {
                if (!first) json << ",";
                json << "[" << b.location.x << "," << b.location.y << "]";
                sent_buildings[i / 64] |= 1U << (i % 64);
                first = false;
            }
        }
        json << "]";

        res.status = 200;
        res.set_content(json.str(), "application/json");
    });

    std::cout << "Server started at http://localhost:" << PORT << std::endl;
    svr.listen("localhost", PORT);

    return 0;
}