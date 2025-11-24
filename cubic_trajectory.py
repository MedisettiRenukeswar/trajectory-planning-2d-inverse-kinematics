from typing import Tuple


def compute_cubic_coeffs(
    q0: float,
    qT: float,
    v0: float = 0.0,
    vT: float = 0.0,
    T: float = 2.0
) -> Tuple[float, float, float, float]:
    """
    Compute cubic polynomial coefficients for:
        q(t) = a0 + a1 t + a2 t^2 + a3 t^3
    with boundary conditions:
        q(0)   = q0
        q(T)   = qT
        q'(0)  = v0
        q'(T)  = vT
    """
    a0 = q0
    a1 = v0
    a2 = (3*(qT - q0) / (T**2)) - (2*v0 + vT) / T
    a3 = (-2*(qT - q0) / (T**3)) + (v0 + vT) / (T**2)
    return a0, a1, a2, a3


def eval_cubic(
    coeffs: Tuple[float, float, float, float],
    t: float
) -> Tuple[float, float]:
    """
    Evaluate cubic at time t.
    Returns (position, velocity).
    """
    a0, a1, a2, a3 = coeffs
    q = a0 + a1 * t + a2 * t**2 + a3 * t**3
    dq = a1 + 2*a2 * t + 3*a3 * t**2
    return q, dq
