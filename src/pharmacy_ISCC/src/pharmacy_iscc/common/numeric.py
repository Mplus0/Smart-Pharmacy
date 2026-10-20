import math


def finite(values):
    return all(not math.isnan(float(v)) and not math.isinf(float(v)) for v in values)


def safe_int(value, default=None):
    try:
        return int(value)
    except Exception:
        return default
