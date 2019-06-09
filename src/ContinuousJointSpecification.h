#pragma once

#include <string>

#include "Interval.h"

namespace NAO {

    struct ContinuousJointSpecification {
    public:

        ContinuousJointSpecification(std::string name, const Interval& valueRange);

        std::string Get_name() const;

        const Interval& Get_value_range() const;

        bool Value_within_boundary(double value) const noexcept;

    private:

        Interval m_value_range;

        std::string m_name;
    };

}