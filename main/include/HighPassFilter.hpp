#pragma once

namespace Hilton {
class HighPassFilter
{
    float alpha;
    float lastValue{0};
    float filteredValue{0};
    float filteredValuePrev{0};

public:
    HighPassFilter(float alpha)
        : alpha(alpha)
    {}
    float filter(float value)
    {
        filteredValuePrev = filteredValue;
        filteredValue = alpha * value + (1 - alpha) * filteredValuePrev;
        return filteredValue;
    }
    float getFilteredValue() { return filteredValue; }
};
} // namespace Hilton