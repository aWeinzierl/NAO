#pragma once

namespace NAO {

    enum class ContinuousJoint {
        HEAD_YAW,
        HEAD_PITCH,

        LEFT_SHOULDER_PITCH,
        LEFT_SHOULDER_ROLL,
        LEFT_ELBOW_YAW,
        LEFT_ELBOW_ROLL,
        LEFT_WRIST_YAW,

        RIGHT_SHOULDER_PITCH,
        RIGHT_SHOULDER_ROLL,
        RIGHT_ELBOW_YAW,
        RIGHT_ELBOW_ROLL,
        RIGHT_WRIST_YAW,
    };

    enum class DiscreteJoint {
        LEFT_HAND,
        RIGHT_HAND,
    };

    enum class Bumper : unsigned char {
        LEFT_BUMPER = 1,
        RIGHT_BUMPER = 0,
        BACK_BUMPER = 2,
    };

    enum class HeadTouch {
        FRONT_HEAD_TOUCH,
        MIDDLE_HEAD_TOUCH,
        BACK_HEAD_TOUCH
    };
}