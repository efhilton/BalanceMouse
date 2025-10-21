#pragma once

#include <string>
#include "SafeEvacException.hpp"

namespace Hilton {
    class NotImplementedException : public SafeEvacException {
    public:
        explicit NotImplementedException(const std::string &msg) : SafeEvacException(msg) {}
    };
} // namespace Hilton
