#pragma once

#include "IOException.hpp"

namespace Hilton {
    class NotFoundException : public IOException {
    public:
        explicit NotFoundException(const std::string &msg) : IOException(msg) {}
    };
} // namespace Hilton
