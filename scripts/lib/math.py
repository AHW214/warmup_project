# https://github.com/Unity-Technologies/UnityCsReference/blob/master/Runtime/Export/Math/Mathf.cs


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def lerp(low: float, high: float, amount: float) -> float:
    return low + (high - low) * clamp(amount, 0.0, 1.0)


def smoothstep(low: float, high: float, amount: float) -> float:
    clamped = clamp(amount, 0.0, 1.0)
    scaled = -2.0 * (clamped ** 3) + 3.0 * (clamped ** 2)
    return high * scaled + low * (1.0 - scaled)


def sign(value: float) -> int:
    return 1 if value > 0 else -1 if value < 0 else 0
