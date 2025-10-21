#pragma once

#include <string>
#include "SafeEvacException.hpp"

namespace SafeEvac {
    class SystemIsBusyException : public SafeEvacException {
    public:
        explicit SystemIsBusyException(const std::string &msg) : SafeEvacException(msg) {}
    };
} // namespace SafeEvac
