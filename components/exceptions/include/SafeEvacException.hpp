#pragma once

#include <stdexcept>

namespace Hilton {
    class SafeEvacException : public std::runtime_error {
    public:
        explicit SafeEvacException(const std::string &msg) : runtime_error(msg) {}
    };
} // namespace Hilton
