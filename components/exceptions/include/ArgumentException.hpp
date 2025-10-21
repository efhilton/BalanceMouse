#pragma once

#include <string>
#include "SafeEvacException.hpp"

namespace Hilton {
    class ArgumentException : public SafeEvacException {
    public:
        explicit ArgumentException(const std::string &msg) : SafeEvacException(msg) {}
    };
} // namespace Hilton
