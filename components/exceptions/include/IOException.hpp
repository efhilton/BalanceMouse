#pragma once

#include "SafeEvacException.hpp"

namespace Hilton {
    class IOException : public SafeEvacException {
    public:
        explicit IOException(const std::string &msg) : SafeEvacException(msg) {}
    };
} // namespace Hilton
