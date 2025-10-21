#pragma once

#include "IOException.hpp"

namespace SafeEvac {
    class TimeoutException : public IOException {
    public:
        explicit TimeoutException(const std::string &msg) : IOException(msg) {}
    };
} // namespace SafeEvac
