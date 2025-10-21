#pragma once

#include "IOException.hpp"

namespace SafeEvac {
    class NetworkException : public IOException {
    public:
        explicit NetworkException(const std::string &msg) : IOException(msg) {}
    };
} // namespace SafeEvac
