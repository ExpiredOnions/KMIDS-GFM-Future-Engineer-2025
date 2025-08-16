#pragma once

// Cardinal directions
enum class Direction
{
    North = 0,
    East = 1,
    South = 2,
    West = 3
};

enum class RotationDirection
{
    Clockwise,
    CounterClockwise
};

// Wall-relative side (inner or outer)
enum class WallSide
{
    Inner = 0,
    Outer = 1
};

// Robot-relative or generic side (left, right, front, back)
enum class RelativeSide
{
    Left = 0,
    Right = 1,
    Front = 2,
    Back = 3
};
