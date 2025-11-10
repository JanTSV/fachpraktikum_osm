#include "string_store.hpp"

size_t NaiveStringStore::get_or_add(std::string value) {
    // Very naive string search in O(m*n)
    for (size_t i = 0; i < _data.size(); i++) {
        if (value == _data[i]) {
            return i;
        }
    }

    _data.push_back(value);
    return _data.size() - 1;
}

size_t MappedStringStore::get_or_add(const std::string& value) {
    auto it = _index.find(value);
    if (it != _index.end()) return it->second;

    size_t idx = _data.size();
    _data.push_back(value);
    _index[_data.back()] = idx;
    return idx;
}


std::string MappedStringStore::get(size_t idx) const {
    return _data[idx];
}