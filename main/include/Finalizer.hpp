#pragma once

namespace Hilton {
class Finalizer
{
    std::function<void()> finalizerCb;

public:
    Finalizer(const std::function<void()> &finalizer)
        : finalizerCb(finalizer)
    {}
    ~Finalizer() { finalizerCb(); }
};
} // namespace Hilton