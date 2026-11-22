"""Programação quadrática pelo método de Hildreth (porte de ``QPhild.m``).

Resolve   min 1/2 x'Hx + f'x   sujeito a   A x <= b.

O método maximiza o dual por subida coordenada a coordenada: os multiplicadores
de Lagrange são atualizados um a um, projetados em lambda >= 0. É simples e não
exige bibliotecas, mas converge devagar quando H é mal condicionada (por exemplo,
com ``psc`` muito grande na variável de folga da CLF).
"""
import numpy as np


def qphild(H, f, A, b, max_iter=200, tol=1e-7):
    H = np.atleast_2d(np.asarray(H, float))
    f = np.asarray(f, float).ravel()
    A = np.atleast_2d(np.asarray(A, float))
    b = np.asarray(b, float).ravel()

    Hinv_f = np.linalg.solve(H, f)
    eta = -Hinv_f                       # solução sem restrições
    if np.all(A @ eta <= b):            # já satisfaz todas as restrições
        return eta

    P = A @ np.linalg.solve(H, A.T)
    d = A @ Hinv_f + b
    lam = np.zeros(len(b))
    for _ in range(max_iter):
        lam_prev = lam.copy()
        for i in range(len(b)):
            w = P[i] @ lam - P[i, i] * lam[i] + d[i]
            lam[i] = max(0.0, -w / P[i, i])
        if (lam - lam_prev) @ (lam - lam_prev) < tol:
            break
    return -Hinv_f - np.linalg.solve(H, A.T @ lam)
