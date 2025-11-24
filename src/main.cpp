#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <optional>
#include <ostream>
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

class KDTree {
    public:
        KDTree() = default;

        void build(std::vector<std::pair<Point, size_t>> points) {
            _nodes.clear();
            _nodes.reserve(points.size());

            struct Task {
                size_t start;
                size_t end;
                size_t depth;
                size_t parent;
                bool is_left;
            };

            std::vector<Task> stack;
            stack.push_back({0, points.size(), 0, std::numeric_limits<size_t>::max(), false});

            while (!stack.empty()) {
                auto task = stack.back();
                stack.pop_back();
                if (task.start >= task.end) continue;

                size_t mid = task.start + (task.end - task.start) / 2;
                size_t axis = task.depth % 2;

                std::nth_element(
                    points.begin() + task.start, points.begin() + mid, points.begin() + task.end,
                    [axis](auto& a, auto& b) { return a.first[axis] < b.first[axis]; }
                );

                Point point = points[mid].first;

                Node node(point,
                          points[mid].second,
                          std::numeric_limits<size_t>::max(),
                          std::numeric_limits<size_t>::max(),
                          axis,
                          Point(point.x, point.y),
                          Point(point.x, point.y));

                size_t node_idx = _nodes.size();
                _nodes.push_back(node);

                if (task.parent != std::numeric_limits<size_t>::max()) {
                    if (task.is_left) _nodes[task.parent].left = node_idx;
                    else _nodes[task.parent].right = node_idx;
                }

                stack.push_back({mid + 1, task.end, task.depth + 1, node_idx, false});
                stack.push_back({task.start, mid, task.depth + 1, node_idx, true});
            }

            compute_bboxes();
        }

        std::vector<size_t> range_search(double min_x, double min_y, double max_x, double max_y) const {
            std::vector<size_t> result;
            if (_nodes.empty()) return result;

            std::array<size_t, 64> stack;
            size_t top = 0;
            stack[top++] = 0;

            while (top > 0) {
                const Node& node = _nodes[stack[--top]];

                if (node.tr.x < min_x || node.bl.x > max_x ||
                    node.tr.y < min_y || node.bl.y > max_y)
                    continue;

                if (node.point.x >= min_x && node.point.x <= max_x &&
                    node.point.y >= min_y && node.point.y <= max_y)
                    result.push_back(node.idx);

                if (node.left != std::numeric_limits<size_t>::max())
                    stack[top++] = node.left;
                if (node.right != std::numeric_limits<size_t>::max())
                    stack[top++] = node.right;
            }

            return result;
        }

        std::optional<size_t> find_nearest(const Point& target) const {
            if (_nodes.empty()) return std::nullopt;

            size_t best_idx = std::numeric_limits<size_t>::max();
            double best_dist_squared = std::numeric_limits<double>::max();

            struct StackItem {
                size_t idx;
                uint8_t depth;
            };

            std::array<size_t, 64> stack;
            size_t top = 0;
            stack[top++] = 0;

            while (top > 0) {
                const Node& node = _nodes[stack[--top]];

                double dx = target.x - node.point.x;
                double dy = target.y - node.point.y;
                double dist_squared = dx * dx + dy * dy;

                if (dist_squared < best_dist_squared) {
                    best_dist_squared = dist_squared;
                    best_idx = node.idx;
                }

                size_t near = std::numeric_limits<size_t>::max();
                size_t far  = std::numeric_limits<size_t>::max();

                double diff = (node.axis == 0 ? dx : dy);
                if (diff < 0) {
                    near = node.left;
                    far  = node.right;
                } else {
                    near = node.right;
                    far  = node.left;
                }

                if (near != std::numeric_limits<size_t>::max()) stack[top++] = near;
                if (far != std::numeric_limits<size_t>::max()) {
                    const Node& far_node = _nodes[far];
                    if (distance_to_box(target, far_node) < best_dist_squared) stack[top++] = far;
                }
            }

            return best_idx;
        }

    private:
        struct Node {
            public:
                Point point;
                size_t idx;
                size_t left;
                size_t right;

                // Bounding box
                Point bl;
                Point tr;

                uint8_t axis;

                Node() = default;
                Node(Point point, size_t idx, size_t left, size_t right, uint8_t axis, Point bl, Point tr)
                    : point(point), idx(idx), left(left), right(right), axis(axis), bl(bl), tr(tr) { }

            private:
                friend class boost::serialization::access;
                template<class Archive>
                void serialize(Archive& ar, const unsigned int /*version*/) {
                    ar & point;
                    ar & idx;
                    ar & left;
                    ar & right;
                    ar & bl;
                    ar & tr;
                    ar & axis;
                }
        };

        std::vector<Node> _nodes;

        void compute_bboxes() {
            if (_nodes.empty()) return;

            std::vector<size_t> order;
            order.reserve(_nodes.size());

            std::array<std::pair<size_t, bool>, 64> stack;
            int top = 0;
            stack[top++] = {0, false};

            while (top > 0) {
                auto [idx, visited] = stack[--top];
                if (idx == std::numeric_limits<size_t>::max()) continue;
                if (visited) {
                    order.push_back(idx);
                    continue;
                }
                stack[top++] = {idx, true};
                if (_nodes[idx].right != std::numeric_limits<size_t>::max()) stack[top++] = {_nodes[idx].right, false};
                if (_nodes[idx].left  != std::numeric_limits<size_t>::max()) stack[top++] = {_nodes[idx].left, false};
            }

            for (size_t idx : order) {
                Node& n = _nodes[idx];
                if (n.left != std::numeric_limits<size_t>::max()) {
                    const Node& l = _nodes[n.left];
                    n.bl.x = std::min(n.bl.x, l.bl.x);
                    n.bl.y = std::min(n.bl.y, l.bl.y);
                    n.tr.x = std::max(n.tr.x, l.tr.x);
                    n.tr.y = std::max(n.tr.y, l.tr.y);
                }
                if (n.right != std::numeric_limits<size_t>::max()) {
                    const Node& r = _nodes[n.right];
                    n.bl.x = std::min(n.bl.x, r.bl.x);
                    n.bl.y = std::min(n.bl.y, r.bl.y);
                    n.tr.x = std::max(n.tr.x, r.tr.x);
                    n.tr.y = std::max(n.tr.y, r.tr.y);
                }
            }
        }

        inline double distance_to_box(const Point& p, const Node& n) const {
            double dx = 0.0;
            if (p.x < n.bl.x) dx = n.bl.x - p.x;
            else if (p.x > n.tr.x) dx = p.x - n.tr.x;

            double dy = 0.0;
            if (p.y < n.bl.y) dy = n.bl.y - p.y;
            else if (p.y > n.tr.y) dy = p.y - n.tr.y;

            return dx * dx + dy * dy;
        }

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _nodes;
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
            _buildings.emplace_back(Point(lat, lon), street_idx, house_number);
        }

        void add_street(const char* name, std::vector<Point> points) override {
            // Skip streets without names
            if (!name) return;

            // Add meta info to _streets
            _streets.emplace_back(_string_store.get_or_add(name), points);
        }

        void add_admin_area(const char* name, std::vector<Point> boundary, uint8_t level) override {
            if (!name) return;
            AdminArea area(_string_store.get_or_add(name), boundary, level);
            double half_width = (area.tr.x - area.bl.x) / 2.0;
            double half_height = (area.tr.y - area.bl.y) / 2.0;
            _areas_max_half_width  = std::max(_areas_max_half_width, half_width);
            _areas_max_half_height = std::max(_areas_max_half_height, half_height);
            _admin_areas.emplace_back(area);
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

        std::string get_buildings_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) const override {
            std::ostringstream json;
            json << "[";
            bool first = true;

            std::vector<size_t> buildings = _buildings_tree.range_search(sw_lat, sw_lon, ne_lat, ne_lon);
            for (auto idx : buildings) {
                if (!first) json << ",";
                const Building& building = _buildings[idx];
                json << "[" 
                     << building.location.x << "," 
                     << building.location.y << ","
                     << (building.street_idx ? "true" : "false") << ","
                     << (building.house_number ? "true" : "false")
                     << "]";
                first = false;
            }
            json << "]";

            return json.str();
        }

        std::string get_streets_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) const override {
            std::ostringstream json;
            json << "[";

            std::vector<size_t> streets = _streets_tree.range_search(sw_lat, sw_lon, ne_lat, ne_lon);
            std::unordered_set<size_t> streets_in_view;
            for (size_t street_idx : streets) {
                streets_in_view.insert(street_idx);
            }

            bool first = true;
            for (size_t s : streets_in_view) {
                const auto &street = _streets[s];

                if (!first) json << ",";
                first = false;

                json << "{\"name\":\""
                    << _string_store.get(street.name_idx)
                    << "\",\"points\":[";

                for (size_t i = 0; i < street.points.size(); i++) {
                    json << "[" << street.points[i].x << "," << street.points[i].y << "]";
                    if (i + 1 < street.points.size()) json << ",";
                }

                json << "]}";
            }
            
            json << "]";
            return json.str();
        }

        std::string get_admin_areas_in_view(double sw_lat, double sw_lon, double ne_lat, double ne_lon) const override {
            std::ostringstream json;
            json << "[";

            Point projected_bl = Point::project_mercator(sw_lat, sw_lon);
            Point projected_tr = Point::project_mercator(ne_lat, ne_lon);
            std::vector<size_t> areas = _admin_areas_tree.range_search(
                projected_bl.x - _areas_max_half_width, 
                projected_bl.y - _areas_max_half_height, 
                projected_tr.x + _areas_max_half_width, 
                projected_tr.y + _areas_max_half_height);
            std::unordered_set<size_t> areas_in_view;
            for (size_t area_idx : areas) {
                areas_in_view.insert(area_idx);
            }

            bool first = true;
            for (size_t a : areas_in_view) {
                const auto &area = _admin_areas[a];

                if (!first) json << ",";
                first = false;

                json << "{\"name\":\""
                    << _string_store.get(area.name_idx)
                    << "\",\"points\":[";

                for (size_t i = 0; i < area.boundary.size(); i++) {
                    json << "[" << area.boundary[i].x << "," << area.boundary[i].y << "]";
                    if (i + 1 < area.boundary.size()) json << ",";
                }

                json << "]}";
            }
            
            json << "]";
            return json.str();
        }

        std::string get_nearest_building(double lat, double lon) const {
            Point target(lat, lon);
            auto nearest_building = _buildings_tree.find_nearest(target);
            if (!nearest_building) return "[]";

            const Building& building = _buildings[*nearest_building];
            std::ostringstream json;
            json << "{";
            json << "\"lat\":" << building.location.x << ",";
            json << "\"lon\":" << building.location.y << ",";

            json << "\"address\":\"" << _string_store.get(building.address);

            if (building.street_idx) {
                json << _string_store.get(*building.street_idx);
            }
            json << "\",";

            if (building.house_number) {
                json << "\"house_number\":\"" << *building.house_number << "\"";
            } else {
                json << "\"house_number\":null";
            }

            json << "}";
            return json.str();
        }

        void preprocess() override {
            std::vector<std::pair<Point, size_t>> pts;
            
            // admin areas kdtree
            std::cout << "\tBuilding admin areas kdtree..." << std::endl;
            auto start = std::chrono::high_resolution_clock::now();
            for (size_t i = 0; i < _admin_areas.size(); i++) {
                const auto& area = _admin_areas[i];
                Point center((area.bl.x + area.tr.x) / 2.0, (area.bl.y + area.tr.y) / 2.0);
                pts.emplace_back(center, i);
            }
            _admin_areas_tree.build(pts);
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << "\tKDtree built " << get_duration(end - start) << std::endl;

            // Point in polygon for each building
            pts.clear();
            std::cout << "\tPoint in polygon test for buildings..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
            for (size_t i = 0; i < _buildings.size(); ++i) {
                Building& building = _buildings[i];
                const Point& p = building.location;
                const Point& projected = Point::project_mercator(p.x, p.y);
                pts.emplace_back(p, i);

                std::vector<size_t> area_candidates = _admin_areas_tree.range_search(
                    projected.x - _areas_max_half_width, 
                    projected.y - _areas_max_half_height, 
                    projected.x + _areas_max_half_width, 
                    projected.y + _areas_max_half_height);
                std::sort(
                    area_candidates.begin(),
                    area_candidates.end(),
                    [this](const size_t a, const size_t b) {
                        return _admin_areas[a].level < _admin_areas[b].level;
                    }
                );

                uint8_t last_lvl = std::numeric_limits<uint8_t>::max();
                std::vector<size_t> address;
                for (const size_t a : area_candidates) {
                    const AdminArea& area = _admin_areas[a];
                    if (area.level == last_lvl) continue;

                    if (area.point_in_polygon_fast(p)) {
                        address.push_back(a);
                        last_lvl = area.level;
                    }
                }

                building.address = build_address(address);
                
                // std::cout << "PiP: " << i << " / " << _buildings.size() << std::endl;
            }
            end = std::chrono::high_resolution_clock::now();
            std::cout << "\tAssigned areas to buldings " << get_duration(end - start) << std::endl;

            // Point in polygon for each building
            // pts.clear();
            // std::cout << "\tPoint in polygon test for buildings..." << std::endl;
            // start = std::chrono::high_resolution_clock::now();
            // std::sort(
            //     _admin_areas.begin(),
            //     _admin_areas.end(),
            //     [](const AdminArea& a, const AdminArea& b) {
            //         return a.level < b.level;
            //     }
            // );

            // for (size_t i = 0; i < _buildings.size(); ++i) {
            //     Building& building = _buildings[i];
            //     const Point& p = building.location;
            //     pts.emplace_back(p, i);

            //     uint8_t last_lvl = std::numeric_limits<uint8_t>::max();
            //     std::vector<size_t> address;
            //     for (size_t a = 0; a < _admin_areas.size(); a++) {
            //         const AdminArea& area = _admin_areas[a];
            //         if (area.level == last_lvl) continue;

            //         if (area.point_in_polygon(p)) {
            //             address.push_back(a);
            //             last_lvl = area.level;
            //         }
            //     }
            //     building.address = build_address(address);

            //     std::cout << "PiP: " << i << " / " << _buildings.size() << std::endl;

            //     //if (i >= 100000) break;
            // }
            // end = std::chrono::high_resolution_clock::now();
            // std::cout << "\tAssigned areas to buldings " << get_duration(end - start) << std::endl;

            // buildings kdtree
            std::cout << "\tBuilding buildings kdtree..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
            _buildings_tree.build(pts);
            end = std::chrono::high_resolution_clock::now();
            std::cout << "\tKDtree built " << get_duration(end - start) << std::endl;

            // streets kdtree
            pts.clear();
            for (size_t i = 0; i < _streets.size(); ++i) {
                for (auto& point : _streets[i].points) {
                    pts.emplace_back(point, i);
                }
            }
            std::cout << "\tBuilding streets kdtree..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
            _streets_tree.build(pts);
            end = std::chrono::high_resolution_clock::now();
            std::cout << "\tKDtree built " << get_duration(end - start) << std::endl;

            // interpolate street names
            std::cout << "\tInterpolating street names for buildings without a street assigned to them..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
            const size_t total = _buildings.size();
            for (size_t i = 0; i < total; i++) {
                auto& building = _buildings[i];
                if (building.street_idx) continue;

                // Check if nearest building has a street name
                // std::optional<size_t> nearest_idx = _buildings_tree.find_nearest(building.location);
                // if (nearest_idx && _buildings[*nearest_idx].street_idx) {
                //     building.street_idx = _buildings[*nearest_idx].street_idx;
                // } else {
                //     // Otherwise find nearest street segment
                //     nearest_idx = _streets_tree.find_nearest(building.location);
                //     if (nearest_idx) {
                //         building.street_idx = _streets[*nearest_idx].name_idx;
                //     }
                // }

                // Check only for nearest street
                std::optional<size_t> nearest_idx = _streets_tree.find_nearest(building.location);
                if (nearest_idx) {
                    building.street_idx = _streets[*nearest_idx].name_idx;
                }

                // std::cout << "\t\t" << i << " / " << total << std::endl;
            }
            end = std::chrono::high_resolution_clock::now();
            std::cout << "\tStreet names assigned " << get_duration(end - start) << std::endl;
        }

        void serialize(const std::string& path) const override {
            std::ofstream ofs(path, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << *this;
        }

    private:
        MappedStringStore _string_store;

        KDTree _buildings_tree;
        std::vector<Building> _buildings;

        KDTree _streets_tree;
        std::vector<Street> _streets;

        KDTree _admin_areas_tree;
        std::vector<AdminArea> _admin_areas;

        double _areas_max_half_width = 0.0;
        double _areas_max_half_height = 0.0;

        size_t build_address(const std::vector<size_t>& addr) {
            std::ostringstream addr_string;
            for (size_t a : addr) {
                const AdminArea& area = _admin_areas[a];
                addr_string << _string_store.get(area.name_idx) << ", ";
            }

            return _string_store.get_or_add(addr_string.str());
        }

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _string_store;
            ar & _buildings_tree;
            ar & _buildings;
            ar & _streets_tree;
            ar & _streets;
            ar & _admin_areas_tree;
            ar & _admin_areas;
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
        auto end_parsing = std::chrono::high_resolution_clock::now();
        std::cout << "Parsing done " << get_duration(end_parsing - start_parsing) << std::endl;
    }
   
    std::cout << "Number of buildings: " << solution.num_buildings() << std::endl;
    std::cout << "Number of streets: " << solution.num_streets() << std::endl;
    std::cout << "Number of admin areas: " << solution.num_admin_areas() << std::endl;

    if (!configuration.in_binary_file) {
        // Preprocessing (only for raw OSM data)
        std::cout << "Preprocessing..." << std::endl;
        auto start_prep = std::chrono::high_resolution_clock::now();

        solution.preprocess();

        auto end_prep = std::chrono::high_resolution_clock::now();
        std::cout << "Preprocessing done " << get_duration(end_prep - start_prep) << std::endl;
    }

    // Serialize preprocessed solution if argument is set
    if (configuration.out_binary_file) {
        std::cout << "Serializing to " << *configuration.out_binary_file << "..." << std::endl;
        auto start_ser = std::chrono::high_resolution_clock::now();

        solution.serialize(*configuration.out_binary_file);

        auto end_ser = std::chrono::high_resolution_clock::now();
        std::cout << "Serialization done " << get_duration(end_ser - start_ser) << std::endl;
    }   

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

    // API endpoint for streets
    svr.Post("/streets", [&solution](const httplib::Request &req, httplib::Response &res) {
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
        res.set_content(solution.get_streets_in_view(sw_lat, sw_lon, ne_lat, ne_lon), "application/json");
    });

    // API endpoint for admin areas
    svr.Post("/admin_areas", [&solution](const httplib::Request &req, httplib::Response &res) {
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
        res.set_content(solution.get_admin_areas_in_view(sw_lat, sw_lon, ne_lat, ne_lon), "application/json");
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