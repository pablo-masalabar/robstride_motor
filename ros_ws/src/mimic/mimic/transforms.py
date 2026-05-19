"""
transforms.py – Per-motor-pair position transform functions for the mimic node.

Each function signature:
    (position: float) -> float

Reference a function by name in config.toml under [transform_map].
If no function is specified for a pair the position is forwarded unchanged.
"""

def passthrough(position: float) -> float:
    return position

def negate(position: float) -> float:
    return -position

def subtract_from_2pi(position: float) -> float:
    return 2 * 3.14159 - position

def subtract_2pi(position: float) -> float:
    return position - 2 * 3.14159

def subtract_from_neg_2pi(position: float) -> float:
    return - 2 * 3.14159 - (position)