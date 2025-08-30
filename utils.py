import math

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    normalized = (x - offset) / span
    normalized = max(0, min(1, normalized))
    return int(normalized * (2**bits - 1))

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    normalized = float(x_int) / (2**bits - 1)
    return normalized * span + offset

def deg_to_rad(deg):
    return deg * math.pi / 180.0

def rad_to_deg(rad):
    return rad * 180.0 / math.pi
