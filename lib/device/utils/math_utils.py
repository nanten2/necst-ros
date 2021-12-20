__all__ = ["clip"]


def clip(value: float, minimum: float, maximum: float) -> float:
    """Limit the `value` to the range [`minimum`, `maximum`]."""
    return min(max(minimum, value), maximum)
