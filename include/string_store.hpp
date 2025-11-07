#pragma once

#include <string>
#include <vector>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/access.hpp>

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