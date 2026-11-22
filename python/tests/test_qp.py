import numpy as np
from cbf_robot import qphild


def test_unconstrained_solution():
    x = qphild(np.eye(2), [-1.0, -1.0], [[1.0, 0.0]], [5.0])      # restrição inativa
    assert np.allclose(x, [1.0, 1.0])


def test_active_constraint():
    x = qphild(np.eye(2), [-1.0, -1.0], [[1.0, 0.0]], [0.5])      # min ||x - (1,1)||^2, x1 <= 0.5
    assert np.allclose(x, [0.5, 1.0], atol=1e-6)


def test_two_constraints():
    A = [[1.0, 0.0], [0.0, 1.0]]
    x = qphild(np.eye(2), [-2.0, -2.0], A, [1.0, 0.5])
    assert np.allclose(x, [1.0, 0.5], atol=1e-6)
