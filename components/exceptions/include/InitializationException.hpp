#pragma once

#include <string>
#include "SafeEvacException.hpp"

namespace SafeEvac {
    class InitializationException : public SafeEvacException {
    public:
        explicit InitializationException(const std::string &msg) : SafeEvacException(msg) {}
    };
} // namespace SafeEvac
