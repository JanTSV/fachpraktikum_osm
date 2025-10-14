#include <iostream>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>

class MyHandler : public osmium::handler::Handler {
public:
    void node(const osmium::Node& node) {
        if (node.tags().has_key("addr:housenumber")) {
            std::cout << "House Node ID: " << node.id()
                      << " | Lat: " << node.location().lat()
                      << " | Lon: " << node.location().lon()
                      << " | Number: " << node.tags()["addr:housenumber"]
                      << std::endl;
        }
    }

    void way(const osmium::Way& way) {
        if (way.tags().has_key("highway")) {
            std::cout << "Street Way ID: " << way.id()
                      << " | Name: " << way.tags()["name"]
                      << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: ./parser <file.osm.pbf>" << std::endl;
        return 1;
    }

    osmium::io::Reader reader(argv[1]);
    MyHandler handler;
    osmium::apply(reader, handler);
    reader.close();

    return 0;
}