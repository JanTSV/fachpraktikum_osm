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

class OSMHandler : public osmium::handler::Handler {
public:
    void buildings(const osmium::Node& node) {
    }

    void streets(const osmium::Way& way) {
    }

    void areas(const osmium::Way& way) {
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
    OSMHandler handler;
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