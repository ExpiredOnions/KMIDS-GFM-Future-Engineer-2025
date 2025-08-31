#pragma once

#include <cmath>
#include <cstdint>

enum class RotationDirection
{
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

// Wall-relative side (inner or outer)
enum class WallSide
{
    INNER = 0,
    OUTER = 1
};

// Robot-relative or generic side (left, right, front, back)
enum class RelativeSide
{
    LEFT = 0,
    RIGHT = 1,
    FRONT = 2,
    BACK = 3
};

// Cardinal directions
class Direction
{
public:
    enum Value : uint8_t
    {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3
    };

    Direction() = default;
    constexpr Direction(Value dir)
        : value(dir) {}

    // Allow switch-case and comparisons
    constexpr operator Value() const {
        return value;
    }

    // Convert to heading in degrees
    constexpr float toHeading() const {
        switch (value) {
        case NORTH:
            return 0.0f;
        case EAST:
            return 90.0f;
        case SOUTH:
            return 180.0f;
        case WEST:
            return 270.0f;
        }
        return 0.0f;  // fallback
    }
    //
    // Construct a Direction from a heading in degrees
    static constexpr Direction fromHeading(float heading) {
        // Normalize to [0, 360)
        float h = std::fmod(heading + 360.0f, 360.0f);
        if (h >= 315.0f || h < 45.0f) return NORTH;
        if (h >= 45.0f && h < 135.0f) return EAST;
        if (h >= 135.0f && h < 225.0f) return SOUTH;
        return WEST;  // 225–315
    }

    /**
     * @brief Get the absolute Direction corresponding to a relative side.
     *
     * @param side RelativeSide (LEFT, RIGHT, FRONT, BACK)
     * @return Direction facing that side relative to the current direction
     */
    constexpr Direction fromRelativeSide(RelativeSide side) const {
        // Map relative side to a 0–3 offset (number of 90° rotations clockwise)
        uint8_t offset = 0;
        switch (side) {
        case RelativeSide::FRONT:
            offset = 0;
            break;
        case RelativeSide::RIGHT:
            offset = 1;
            break;
        case RelativeSide::BACK:
            offset = 2;
            break;
        case RelativeSide::LEFT:
            offset = 3;
            break;
        }
        return Direction(static_cast<Value>((static_cast<uint8_t>(value) + offset) % 4));
    }

private:
    Value value;
};
