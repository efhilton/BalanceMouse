#pragma once

namespace Hilton {
    class HighPassFilter {
        float alpha;
        float lastValue{0};
        float filteredValue{0};
        float filteredValuePrev{0};
        bool initialized{false};

    public:
        explicit HighPassFilter(const float alpha)
            : alpha(alpha) {
        }

        float filter(const float value) {
            if (!initialized) {
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

        [[nodiscard]] float getFilteredValue() const { return filteredValue; }
    };
} // namespace Hilton
