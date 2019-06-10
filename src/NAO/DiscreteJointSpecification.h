#pragma once

#include <unordered_set>
#include <string>
#include <limits>
#include <cmath>


namespace NAO {

    struct ApproximatelyEqual {
        template<typename KeyType>
        bool operator()(const KeyType &lhs, const KeyType &rhs) const {
            return std::abs(lhs - rhs) < std::numeric_limits<KeyType>::epsilon();
        }
    };

    struct DiscreteJointSpecification {
    public:
        DiscreteJointSpecification(std::string name, std::unordered_set<
                double,
                std::unordered_set<double>::hasher,
                ApproximatelyEqual,
                std::unordered_set<double>::allocator_type> valueSet, uint index);


        const std::string &Get_name() const;

        const std::unordered_set<
                double,
                std::unordered_set<double>::hasher,
                ApproximatelyEqual,
                std::unordered_set<double>::allocator_type> &Get_value_set() const;

        bool Value_valid(double value) const noexcept {
            return m_value_set.find(value) != m_value_set.end();
        }

        uint Get_index() const noexcept;

    private:
        std::unordered_set<
                double,
                std::unordered_set<double>::hasher,
                ApproximatelyEqual,
                std::unordered_set<double>::allocator_type> m_value_set;

        std::string m_name;

        uint m_index;
    };

}