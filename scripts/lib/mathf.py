"""
Math helper functions inspired by Unity's mathf module, see:
https://github.com/Unity-Technologies/UnityCsReference/blob/master/Runtime/Export/Math/Mathf.cs
"""


def clamp(value: float, low: float, high: float) -> float:
    """
    Clamp a value between a low and high value.
    """
    return min(high, max(low, value))


def lerp(low: float, high: float, amount: float) -> float:
    """
    Linearly interpolate between a low and high value.

    @param `amount`: The amount by which to interpolate, in the range [0, 1].
    """
    return low + (high - low) * clamp(amount, 0.0, 1.0)


def smoothstep(low: float, high: float, amount: float) -> float:
    """
    Smoothly interpolate between a low and high value.

    @param `amount`: The amount by which to interpolate, in the range [0, 1].
    """
    clamped = clamp(amount, 0.0, 1.0)
    scaled = -2.0 * (clamped ** 3) + 3.0 * (clamped ** 2)
    return high * scaled + low * (1.0 - scaled)


def sign(value: float) -> int:
    """
    Get the sign of the value.

    @return: 1 if `value` is positive, -1 if `value` is negative, or 0 otherwise.
    """
    return 1 if value > 0 else -1 if value < 0 else 0


def zero_under(low: float, value: float) -> float:
    """
    Return zero if `value` is less than `low`, or return `value`.
    """
    return 0.0 if value < low else value


def zero_abs_under(low: float, value: float) -> float:
    """
    Return zero if the absolute value of `value` is less than `low`, or return
    `value`.
    """
    return 0.0 if abs(value) < low else value
