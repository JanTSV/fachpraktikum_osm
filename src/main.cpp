#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>
#include <fstream>
#include <unordered_set>
#include <unordered_map>

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
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>

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

struct SuffixArrayEntry {
    public:
        size_t string_id;
        size_t offset;
        size_t building_idx;

        bool operator<(const SuffixArrayEntry& other) const {
            return std::tie(string_id, offset) < std::tie(other.string_id, other.offset);
        }

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & string_id;
            ar & offset;
            ar & building_idx;
        }
};

class SuffixArray {
    public:
        SuffixArray(MappedStringStore& store) : _string_store(store) {}

        void add_building(size_t building_idx, const Building& building) {
            auto add_all_suffixes = [&](size_t string_id) {
                std::string_view s = _string_store.get(string_id);
                for (size_t i = 0; i < s.size(); i++) {
                    _entries.push_back({ string_id, i, building_idx });
                }
            };

            add_all_suffixes(building.address);
            if (building.shop_name) add_all_suffixes(*building.shop_name);
            if (building.street_idx) add_all_suffixes(*building.street_idx);
            if (building.house_number) {
                const std::string house_str = std::to_string(*building.house_number);
                size_t house_str_idx = _string_store.get_or_add(house_str);
                add_all_suffixes(house_str_idx);
            }
        }

        void build() {
            std::sort(_entries.begin(), _entries.end(),
                [this](const SuffixArrayEntry& a, const SuffixArrayEntry& b) {
                    std::string_view sa = _string_store.get(a.string_id).substr(a.offset);
                    std::string_view sb = _string_store.get(b.string_id).substr(b.offset);
                    return sa < sb;
                });
        }

        std::unordered_set<size_t> search_buildings(const std::string& query) const {
            std::unordered_set<size_t> result;

            auto lower = std::lower_bound(_entries.begin(), _entries.end(), query,
                [this](const SuffixArrayEntry& s, const std::string& q) {
                    std::string_view sv = _string_store.get(s.string_id).substr(s.offset);
                    return sv.compare(0, q.size(), q) < 0;
                });

            auto upper = std::upper_bound(_entries.begin(), _entries.end(), query,
                [this](const std::string& q, const SuffixArrayEntry& s) {
                    std::string_view sv = _string_store.get(s.string_id).substr(s.offset);
                    return q.compare(0, q.size(), sv.substr(0, q.size())) < 0;
                });

            for (auto it = lower; it != upper; ++it) {
                result.insert(it->building_idx);
            }

            return result;
        }

    private:
        MappedStringStore& _string_store;
        std::vector<SuffixArrayEntry> _entries;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _entries;
        }
};

struct SuffixIndirect {
    size_t string_id;  // String store index
    size_t offset;  // Position in string
    size_t length;  // Length of suffix

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & string_id;
            ar & offset;
            ar & length;
        }
};

struct SuffixTreeEdge {
    SuffixIndirect suffix;
    size_t child;

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & suffix;
            ar & child;
        }
};

struct SuffixTreeNode {
    std::unordered_map<char, SuffixTreeEdge> edges;
    std::unordered_set<size_t> buildings;

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & edges;
            ar & buildings;
        }
};

class SuffixTree {
    public:
        SuffixTree(MappedStringStore& string_store) : _string_store(string_store) {
            // Create root node
            _nodes.push_back(SuffixTreeNode{ });
        }

        void add_building(size_t building_idx, const Building& building) {
            // Build suffixes for all strings in the building
            std::vector<SuffixIndirect> suffixes = build_address_suffixes(building);
            for (size_t i = 0; i < suffixes.size(); i++) {
                insert_suffix(building_idx, suffixes[i]);
            }
        }

        std::unordered_set<size_t> search_buildings(std::string_view query) const {
            size_t node_idx = 0;
            size_t q_pos = 0;

            while (q_pos < query.size()) {
                char c = query[q_pos];
                const auto& edges = _nodes[node_idx].edges;
                auto it = edges.find(c);
                if (it == edges.end()) return {};

                const SuffixTreeEdge& edge = it->second;
                std::string_view edge_str = _string_store.get(edge.suffix.string_id);
                size_t edge_len = edge.suffix.length;

                size_t i = 0;
                while (i < edge_len && q_pos < query.size()) {
                    if (edge_str[edge.suffix.offset + i] != query[q_pos]) return {};
                    i++;
                    q_pos++;
                }

                node_idx = edge.child;
            }

            return collect_buildings(node_idx);
        }

    private:
        MappedStringStore& _string_store;
        std::vector<SuffixTreeNode> _nodes;

        std::unordered_set<size_t> collect_buildings(size_t node_idx) const {
            // Only leaves store buildings, so traverse subtree to collect them
            std::unordered_set<size_t> result;
            std::vector<size_t> stack = { node_idx };

            while (!stack.empty()) {
                size_t idx = stack.back();
                stack.pop_back();

                const auto& node = _nodes[idx];
                result.insert(node.buildings.begin(), node.buildings.end());

                for (const auto& [_, edge] : node.edges) {
                    stack.push_back(edge.child);
                }
            }

            return result;
        }

        std::vector<SuffixIndirect> build_address_suffixes(const Building& building) {
            std::vector<SuffixIndirect> suffixes;

            auto add_all_suffixes = [&](size_t string_id) {
                std::string_view s = _string_store.get(string_id);
                for (size_t i = 0; i < s.size(); i++) {
                    suffixes.push_back({ string_id, i, s.size() - i });
                }
            };

            add_all_suffixes(building.address);

            if (building.shop_name) add_all_suffixes(*building.shop_name);
            if (building.street_idx) add_all_suffixes(*building.street_idx);
            if (building.house_number) {
                const std::string house_str = std::to_string(*building.house_number);
                add_all_suffixes(_string_store.get_or_add(house_str));
            }

            return suffixes;
        }

        void insert_suffix(size_t building_idx, const SuffixIndirect& suffix) {
            size_t node_idx = 0;
            size_t offset = 0;

            std::string_view str = _string_store.get(suffix.string_id);

            while (offset < suffix.length) {
                char c = str[suffix.offset + offset];
                auto it = _nodes[node_idx].edges.find(c);

                if (it == _nodes[node_idx].edges.end()) {
                    // Create new leaf node
                    size_t new_node_idx = _nodes.size();
                    _nodes.push_back(SuffixTreeNode{});
                    SuffixIndirect remaining_suffix{ suffix.string_id, suffix.offset + offset, suffix.length - offset };
                    _nodes[node_idx].edges[c] = { remaining_suffix, new_node_idx };
                    _nodes[new_node_idx].buildings.insert(building_idx); // Only leaves store buildings
                    return;
                }

                SuffixTreeEdge& edge = it->second;
                std::string_view edge_str = _string_store.get(edge.suffix.string_id);
                size_t match_len = 0;

                // Find longest common prefix
                while (match_len < edge.suffix.length && offset + match_len < suffix.length &&
                    str[suffix.offset + offset + match_len] == edge_str[edge.suffix.offset + match_len]) {
                    match_len++;
                }

                if (match_len < edge.suffix.length) {
                    // Split edge
                    size_t old_child = edge.child;

                    SuffixIndirect remaining_edge{ edge.suffix.string_id, edge.suffix.offset + match_len, edge.suffix.length - match_len };
                    size_t split_node = _nodes.size();
                    _nodes.push_back(SuffixTreeNode{});
                    _nodes[split_node].edges[_string_store.get(remaining_edge.string_id)[remaining_edge.offset]] = { remaining_edge, old_child };

                    edge.suffix.length = match_len;
                    edge.child = split_node;
                }

                offset += match_len;
                node_idx = edge.child;

                if (offset >= suffix.length) {
                    // Insert leaf here if fully consumed suffix
                    _nodes[node_idx].buildings.insert(building_idx);
                    return;
                }
            }
        }

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _string_store;
            ar & _nodes;
        }
};

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
                    : point(point), idx(idx), left(left), right(right), bl(bl), tr(tr), axis(axis) { }

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
        // TODO: KDSolution() : _suffix_tree(_string_store) { }
        KDSolution() : _suffix_array(_string_store) { }

        void add_building(double lat, double lon, const char* street, std::optional<HouseNumber> house_number, const char* shop_name) override {
            // Add meta info to _buildings
            std::optional<size_t> street_idx;
            if (street) {
                street_idx = _string_store.get_or_add(street);
            }
            std::optional<size_t> shop_name_idx;
            if (shop_name) {
                shop_name_idx = _string_store.get_or_add(shop_name);
            }
            _buildings.emplace_back(Point(lat, lon), street_idx, house_number, shop_name_idx);
        }

        void add_street(const char* name, std::vector<Point> points) override {
            // Skip streets without names
            if (!name) return;

            // Add meta info to _streets
            _streets.emplace_back(_string_store.get_or_add(name), points);
        }

        void add_admin_area(const char* name, std::vector<std::vector<Point>> boundaries, uint8_t level) override {
            if (!name) return;
            AdminArea area(_string_store.get_or_add(name), boundaries, level);
            _hierarchical_admin_areas[level].push_back(area);
        }

        size_t num_buildings() const override {
            return _buildings.size();
        }

        size_t num_streets() const override {
            return _streets.size();
        }

        size_t num_admin_areas() const override {
            size_t num = 0;
            for (const auto& [_, areas] : _hierarchical_admin_areas) {
                num += areas.size();
            }
            return num;
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

        std::string get_admin_areas_in_view(double sw_lat, double sw_lon,
                                            double ne_lat, double ne_lon) const override {
            std::ostringstream json;
            json << "[";

            Point projected_bl = Point::project_mercator(sw_lat, sw_lon);
            Point projected_tr = Point::project_mercator(ne_lat, ne_lon);

            bool first = true;

            for (const auto& [level, area_level] : _hierarchical_admin_areas_trees) {
                std::unordered_set<size_t> areas_in_view;

                std::vector<size_t> candidates = area_level.tree.range_search(
                    projected_bl.x - area_level.max_half_width,
                    projected_bl.y - area_level.max_half_height,
                    projected_tr.x + area_level.max_half_width,
                    projected_tr.y + area_level.max_half_height
                );

                for (size_t idx : candidates) {
                    areas_in_view.insert(idx);
                }

                for (size_t a : areas_in_view) {
                    const AdminArea& area = _hierarchical_admin_areas.at(level)[a];

                    if (!first) json << ",";
                    first = false;

                    json << "{";
                    json << "\"name\":\"" << _string_store.get(area.name_idx) << "\",";
                    json << "\"level\":" << int(level) << ",";
                    json << "\"polygons\":[";

                    for (size_t r = 0; r < area.boundaries.size(); ++r) {
                        if (r > 0) json << ",";
                        json << "[";

                        const auto& ring = area.boundaries[r];
                        for (size_t i = 0; i < ring.size(); ++i) {
                            json << "[" << ring[i].x << "," << ring[i].y << "]";
                            if (i + 1 < ring.size()) json << ",";
                        }

                        json << "]";
                    }

                    json << "]";
                    json << "}";
                }
            }

            json << "]";
            return json.str();
        }

        std::string get_nearest_building(double lat, double lon) const override {
            Point target(lat, lon);
            auto nearest_building = _buildings_tree.find_nearest(target);
            if (!nearest_building) return "[]";

            const Building& building = _buildings[*nearest_building];
            std::ostringstream json;
            json << "{";
            json << "\"lat\":" << building.location.x << ",";
            json << "\"lon\":" << building.location.y << ",";

            json << "\"address\":\"" << _string_store.get(building.address);

            if (building.shop_name) {
                json << _string_store.get(*building.shop_name) << ", ";
            }
            if (building.street_idx) {
                json << _string_store.get(*building.street_idx);
            }
            json << "\",";

            if (building.house_number) {
                json << "\"house_number\":\"" << *building.house_number << "\",";
            } else {
                json << "\"house_number\":null,";
            }
            json << "\"distance\":" << building.location.haversine_distance(target);

            json << "}";
            return json.str();
        }

        std::string get_nearest_street(double lat, double lon) const override {
            Point target(lat, lon);
            auto nearest_street = _streets_tree.find_nearest(target);
            if (!nearest_street) return "[]";

            const Street& street = _streets[*nearest_street];
            std::ostringstream json;
            json << "{";
            json << "\"name\":\"" << _string_store.get(street.name_idx) << "\",";
            json << "\"points\":[";
            double min_distance = std::numeric_limits<double>::max();
            for (size_t i = 0; i < street.points.size(); i++) {
                json << "[" << street.points[i].x << "," << street.points[i].y << "]";
                if (i + 1 < street.points.size()) json << ",";
                min_distance = std::min(min_distance, street.points[i].haversine_distance(target));
            }
            json << "],";
            json << "\"distance\":" << min_distance;
            json << "}";
            return json.str();
        }

        std::string get_nearest_admin_area(double lat, double lon, int filter, int expected_level) const override {
            Point target(lat, lon);
            Point projected_target = Point::project_mercator(lat, lon);

            const AdminArea* nearest_area = nullptr;
            double min_distance = std::numeric_limits<double>::max();
            bool done = false;

            // Iterate over all (filtered) areas by level (higher level = more local area)
            // If clicked point is within an area, return it isntantly, otherwise keep track
            // of the nearest area.
            for (auto it = _hierarchical_admin_areas_trees.rbegin();
                it != _hierarchical_admin_areas_trees.rend() && !done;
                ++it) {

                uint8_t level = it->first;
                const auto& area_level = it->second;
                
                // Filter out unexpected levels
                if (filter && level != expected_level) continue;

                std::vector<size_t> area_candidates =
                    area_level.tree.range_search(
                        projected_target.x - area_level.max_half_width,
                        projected_target.y - area_level.max_half_height,
                        projected_target.x + area_level.max_half_width,
                        projected_target.y + area_level.max_half_height
                    );

                for (size_t a : area_candidates) {
                    const AdminArea& area = _hierarchical_admin_areas.at(level).at(a);
                    if (area.point_in_polygon(target)) {
                        nearest_area = &area;
                        min_distance = 0.0;
                        done = true;
                        break;
                    }
                    for (const auto& ring : area.boundaries) {
                        for (const Point& p : ring) {
                            double distance = p.haversine_distance(target);
                            if (distance < min_distance) {
                                min_distance = distance;
                                nearest_area = &area;
                            }
                        }
                    }
                }
            }

            if (!nearest_area)
                return "[]";

            std::ostringstream json;
            json << "{";
            json << "\"name\":\"" << _string_store.get(nearest_area->name_idx) << "\",";
            json << "\"polygons\":[";

            for (size_t i = 0; i < nearest_area->boundaries.size(); ++i) {
                const auto& ring = nearest_area->boundaries[i];
                json << "[";
                for (size_t j = 0; j < ring.size(); ++j) {
                    json << "[" << ring[j].x << "," << ring[j].y << "]";
                    if (j + 1 < ring.size()) json << ",";
                }
                json << "]";
                if (i + 1 < nearest_area->boundaries.size()) json << ",";
            }

            json << "],";
            json << "\"distance\":" << min_distance;
            json << "}";

            return json.str();
        }

        std::string search_buildings(std::string& query) const override {
            std::ostringstream json;
            json << "[";
            // TODO: for (size_t building_idx : _suffix_tree.search_buildings(query)) {
            for (size_t building_idx : _suffix_array.search_buildings(query)) {
                const Building& building = _buildings[building_idx];
                json << "{";
                json << "\"lat\":" << building.location.x << ",";
                json << "\"lon\":" << building.location.y << ",";

                json << "\"address\":\"" << _string_store.get(building.address);

                if (building.shop_name) {
                    json << _string_store.get(*building.shop_name) << ", ";
                }
                if (building.street_idx) {
                    json << _string_store.get(*building.street_idx);
                }
                json << "\",";

                if (building.house_number) {
                    json << "\"house_number\":\"" << *building.house_number << "\",";
                } else {
                    json << "\"house_number\":null";
                }
                // TODO: json << ",";
                // TODO: json << "\"distance\":" << building.location.haversine_distance(target);

                json << "}";
            }
            json << "]";

            return json.str();
        }

        void preprocess() override {
            std::vector<std::pair<Point, size_t>> pts;

            // admin areas hierarchical
            std::cout << "\tBuilding hierarchical admin areas kdtrees..." << std::endl;
            auto start = std::chrono::high_resolution_clock::now();
            for (const auto& [level, areas] : _hierarchical_admin_areas) {
                double max_half_width = 0.0;
                double max_half_height = 0.0;

                for (size_t i = 0; i < areas.size(); i++) {
                    const auto& area = areas[i];
                    Point center((area.bl.x + area.tr.x) / 2.0, (area.bl.y + area.tr.y) / 2.0);
                    pts.emplace_back(center, i);

                    const double half_width = (area.tr.x - area.bl.x) / 2.0;
                    const double half_height = (area.tr.y - area.bl.y) / 2.0;
                    max_half_width = std::max(max_half_width, half_width);
                    max_half_height = std::max(max_half_height, half_height);
                }

                AdminAreaLevel area_level(max_half_width, max_half_height, pts);
                _hierarchical_admin_areas_trees[level] = area_level;
                pts.clear();
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << "\tHierarchical admin areas kdtrees built " << get_duration(end - start) << std::endl;

            // PiP for each building using hierarchical admin areas
            pts.clear();
            std::cout << "\tPoint in polygon test for buildings using hierarchical admin areas..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
            for (size_t i = 0; i < _buildings.size(); ++i) {
                Building& building = _buildings[i];
                const Point& p = building.location;
                const Point& projected = Point::project_mercator(p.x, p.y);

                pts.emplace_back(p, i);  // Add to pts for buildings later

                std::vector<size_t> address;
                for (const auto& [level, area_level] : _hierarchical_admin_areas_trees) {
                    std::vector<size_t> area_candidates = area_level.tree.range_search(
                        projected.x - area_level.max_half_width, 
                        projected.y - area_level.max_half_height, 
                        projected.x + area_level.max_half_width, 
                        projected.y + area_level.max_half_height);

                    for (const size_t a : area_candidates) {
                        const AdminArea& area = _hierarchical_admin_areas[level][a];
                        if (area.point_in_polygon(p)) {
                            address.push_back(area.name_idx);
                            break;
                        }
                    }
                }
                
                building.address = build_address(address);
                // std::cout << "PiP: " << i << " / " << _buildings.size() << std::endl;
            }
            end = std::chrono::high_resolution_clock::now();
            std::cout << "\tAssigned areas to buildings " << get_duration(end - start) << std::endl;

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

            // Build suffix tree
            // TODO: std::cout << "\tBuilding suffix tree..." << std::endl;
            // start = std::chrono::high_resolution_clock::now();

            // for (size_t i = 0; i < _buildings.size(); i++) {
            //     std::cout << "\t" << i << " / " << _buildings.size() << std::endl;
            //     const Building& building = _buildings[i];
            //     _suffix_tree.add_building(i, building);
            // }

            // end = std::chrono::high_resolution_clock::now();
            // std::cout << "\tBuilt suffix tree " << get_duration(end - start) << std::endl;
            std::cout << "\tBuilding suffix array..." << std::endl;
            start = std::chrono::high_resolution_clock::now();

            for (size_t i = 0; i < _buildings.size(); ++i) {
                std::cout << "\t" << i << " / " << _buildings.size() << std::endl;
                _suffix_array.add_building(i, _buildings[i]);
            }

            _suffix_array.build();
            end = std::chrono::high_resolution_clock::now();
            std::cout << "\tSuffix array built " << get_duration(end - start) << std::endl;
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

        struct AdminAreaLevel {
            public:
                double max_half_width;
                double max_half_height;
                KDTree tree;

                AdminAreaLevel() : max_half_width(0.0), max_half_height(0.0), tree() { }

                AdminAreaLevel(double max_half_width, double max_half_height, std::vector<std::pair<Point, size_t>> points)
                    : max_half_width(max_half_width), max_half_height(max_half_height), tree() {
                        tree.build(points);
                }

            private:
                friend class boost::serialization::access;
                template<class Archive>
                void serialize(Archive& ar, const unsigned int /*version*/) {
                    ar & max_half_width;
                    ar & max_half_height;
                    ar & tree;
                }
        };

        std::map<uint8_t, AdminAreaLevel> _hierarchical_admin_areas_trees;
        std::map<uint8_t, std::vector<AdminArea>> _hierarchical_admin_areas;

        size_t build_address(const std::vector<size_t>& addr) {
            std::ostringstream addr_string;
            for (size_t a : addr) {
                addr_string << _string_store.get(a) << ", ";
            }

            return _string_store.get_or_add(addr_string.str());
        }

        // TODO: SuffixTree _suffix_tree;
        SuffixArray _suffix_array;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _string_store;
            ar & _buildings_tree;
            ar & _buildings;
            ar & _streets_tree;
            ar & _streets;
            ar & _hierarchical_admin_areas_trees;
            // TODO: ar & _suffix_tree;
            ar & _suffix_array;
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

    // API endpoints for reverse geocoder
    svr.Post("/nearest_building", [&solution](const httplib::Request &req, httplib::Response &res) {
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

    svr.Post("/nearest_street", [&solution](const httplib::Request &req, httplib::Response &res) {
        double lat, lon;
        bool ok = (sscanf(req.body.c_str(), R"({"lat":%lf,"lon":%lf})", &lat, &lon) == 2);
        if (!ok) {
            res.status = 400;
            res.set_content("Invalid JSON format", "text/plain");
            return;
        }
        res.status = 200;
        res.set_content(solution.get_nearest_street(lat, lon), "application/json");
    });

    svr.Post("/nearest_admin_area", [&solution](const httplib::Request &req, httplib::Response &res) {
        double lat, lon;
        int filter, expected_level;
        bool ok = (sscanf(req.body.c_str(), R"({"lat":%lf,"lon":%lf,"filter":%d,"expectedLevel":%d})", &lat, &lon, &filter, &expected_level) == 4);
        if (!ok) {
            res.status = 400;
            res.set_content("Invalid JSON format", "text/plain");
            return;
        }
        res.status = 200;
        res.set_content(solution.get_nearest_admin_area(lat, lon, filter, expected_level), "application/json");
    });

    // API endpoint for geocoder
    svr.Post("/search_buildings", [&solution](const httplib::Request& req, httplib::Response& res) {
        char query[1024];

        bool ok = (sscanf(req.body.c_str(), R"({"query":"%1023[^"]"})", query) == 1);
        if (!ok) {
            res.status = 400;
            res.set_content("Invalid JSON", "text/plain");
            return;
        }

        std::string q(query);
        std::cout << "QUERY: " << q << std::endl;

        res.status = 200;
        
        res.set_content(
            solution.search_buildings(q),
            "application/json"
        );
    });

    std::cout << "Server started at http://localhost:" << configuration.port << std::endl;
    svr.listen("localhost", configuration.port);

    return 0;
}