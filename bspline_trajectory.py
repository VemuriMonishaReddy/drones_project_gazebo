#!/usr/bin/env python3
"""Cubic B-spline trajectory generator using Cox-de Boor recursion."""
import numpy as np

def open_uniform_knots(n_ctrl, degree=3):
    n_knots = n_ctrl + degree + 1
    interior = n_knots - 2*(degree+1)
    if interior <= 0:
        return np.concatenate([np.zeros(degree+1), np.ones(degree+1)])
    mid = np.linspace(0, 1, interior+2)[1:-1]
    return np.concatenate([np.zeros(degree+1), mid, np.ones(degree+1)])

def basis(i, degree, u, knots):
    if degree == 0:
        return 1.0 if ((knots[i] <= u < knots[i+1]) or (u == knots[-1] and knots[i+1] == knots[-1])) else 0.0
    a = knots[i+degree] - knots[i]
    b = knots[i+degree+1] - knots[i+1]
    left = ((u-knots[i])/a * basis(i, degree-1, u, knots)) if a > 1e-12 else 0.0
    right = ((knots[i+degree+1]-u)/b * basis(i+1, degree-1, u, knots)) if b > 1e-12 else 0.0
    return left + right

def evaluate(control_points, u, degree=3):
    cp = np.asarray(control_points, dtype=float)
    knots = open_uniform_knots(len(cp), degree)
    return sum(basis(i, degree, float(np.clip(u,0,1)), knots) * cp[i] for i in range(len(cp)))

def sample(control_points, count=500, degree=3):
    us = np.linspace(0, 1, count)
    pts = np.array([evaluate(control_points, u, degree) for u in us])
    return us, pts
