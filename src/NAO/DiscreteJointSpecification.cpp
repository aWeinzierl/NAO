#include "DiscreteJointSpecification.h"

namespace NAO {

    const std::string &DiscreteJointSpecification::Get_name() const {
        return m_name;
    }

    const std::unordered_set<
            double,
            std::unordered_set<double>::hasher,
            ApproximatelyEqual,
            std::unordered_set<double>::allocator_type> &DiscreteJointSpecification::Get_value_set() const {
        return m_value_set;
    }

    DiscreteJointSpecification::DiscreteJointSpecification(std::string name, std::unordered_set<
            double,
            std::unordered_set<double>::hasher,
            ApproximatelyEqual,
            std::unordered_set<double>::allocator_type> valueSet, uint index)
            : m_name(std::move(name)), m_value_set(std::move(valueSet)), m_index(index) {

    }

    uint DiscreteJointSpecification::Get_index() const noexcept {
        return m_index;
    }
}