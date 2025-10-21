#pragma once

#include "NotFoundException.hpp"

namespace SafeEvac {
    class FileNotFoundException : public NotFoundException {
    public:
        explicit FileNotFoundException(const std::string &msg) : NotFoundException(msg) {}
    };
} // namespace SafeEvac
