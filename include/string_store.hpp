#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/unordered_map.hpp>

struct NaiveStringStore {
    public:
        size_t get_or_add(std::string value);

    private:
        std::vector<std::string> _data;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _data;
        }
};

struct MappedStringStore {
    public:
        size_t get_or_add(const std::string& value);
        std::string get(size_t idx) const;

    private:
        std::vector<std::string> _data;
        std::unordered_map<std::string, size_t> _index;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) {
            ar & _data;
            ar & _index;
        }
};