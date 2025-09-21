#pragma once

namespace Hilton {
class HighPassFilter
{
    float alpha;
    float lastValue{0};
    float filteredValue{0};
    float filteredValuePrev{0};
    bool initialized{false};

public:
    HighPassFilter(float alpha)
        : alpha(alpha)
    {}
    float filter(float value)
    {
        if (!initialized)
        {
            lastValue = value;
            filteredValue = value;
            filteredValuePrev = value;
            initialized = true;
            return value;
        }
        filteredValuePrev = filteredValue;
        filteredValue = alpha * value + (1 - alpha) * filteredValuePrev;
        return filteredValue;
    }
    float getFilteredValue() { return filteredValue; }
};
} // namespace Hilton