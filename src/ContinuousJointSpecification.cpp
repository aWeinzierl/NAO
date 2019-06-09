#include "ContinuousJointSpecification.h"

std::string NAO::ContinuousJointSpecification::Get_name() const {
    return m_name;
}

const NAO::Interval &NAO::ContinuousJointSpecification::Get_value_range() const {
    return m_value_range;
}

NAO::ContinuousJointSpecification::ContinuousJointSpecification(std::string name, const NAO::Interval &valueRange)
:m_name(std::move(name)), m_value_range(valueRange){}

bool NAO::ContinuousJointSpecification::Value_within_boundary(double value) const noexcept {
    return m_value_range.lowerLimit < value && m_value_range.upperLimit > value;
}
