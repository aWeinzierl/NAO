
#pragma once

namespace NAO {
    struct Interval {
        constexpr Interval(double lowerLimit, double upperLimit) : lowerLimit(lowerLimit), upperLimit(upperLimit) {}

        float lowerLimit;
        float upperLimit;
    };
}