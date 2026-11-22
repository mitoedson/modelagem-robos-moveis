"""CLF, CBFs e o QP que combina seguimento e segurança.

Modelo (monociclo):  x' = v cos(th),  y' = v sin(th),  th' = w   (f = 0, g = [[c,0],[s,0],[0,1]])

Variáveis do QP: z = [v, w, delta], com delta a folga da CLF.
"""
from dataclasses import dataclass
from collections import namedtuple
import numpy as np
from .qp import qphild

# Referência: posição, velocidade, orientação e taxa de orientação desejadas
Ref = namedtuple("Ref", "xd yd xd_dot yd_dot thd thd_dot")


@dataclass
class QPParams:
    l: float = 0.2        # distância do ponto de controle à frente do eixo [m]
    rs: float = 0.3       # raio da região de segurança [m]
    psc: float = 100.0    # peso da folga da CLF
    c: float = 0.3        # taxa de decaimento da CLF
    lam: float = 10.0     # ganho da CBF (alpha(h) = lam*h)
    vmax: float = 2.0     # limite de |v| e |w|
    ctheta: float = 1.0   # peso do erro de ângulo em V


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def pose_errors(x, y, th, ref):
    ex = np.cos(th) * (ref.xd - x) + np.sin(th) * (ref.yd - y)
    ey = -np.sin(th) * (ref.xd - x) + np.cos(th) * (ref.yd - y)
    return ex, ey, ref.thd - th


def clf_terms(x, y, th, ref, ctheta=1.0):
    """V, LgV e Vt de V = (ex^2+ey^2)/2 + ctheta (1-cos e_theta)."""
    ex, ey, et = pose_errors(x, y, th, ref)
    V = 0.5 * (ex ** 2 + ey ** 2) + ctheta * (1 - np.cos(et))
    LgV = np.array([x * np.cos(th) - ref.xd * np.cos(th) + y * np.sin(th) - ref.yd * np.sin(th),
                    ctheta * np.sin(th - ref.thd)])
    Vt = (ref.xd * ref.xd_dot - x * ref.xd_dot - y * ref.yd_dot + ref.yd * ref.yd_dot
          - ctheta * ref.thd_dot * np.sin(th - ref.thd))
    return V, LgV, Vt


def cbf_terms(x, y, th, obs, l, rs):
    """h_i e Lg h_i para cada obstáculo, com a barreira no ponto de controle."""
    px, py = x + l * np.cos(th), y + l * np.sin(th)
    h, Lgh = [], []
    for ox, oy in obs:
        dx, dy = px - ox, py - oy
        h.append(dx ** 2 + dy ** 2 - rs ** 2)
        Lgh.append([2 * dx * np.cos(th) + 2 * dy * np.sin(th),
                    2 * l * (dy * np.cos(th) - dx * np.sin(th))])
    return np.array(h), np.array(Lgh).reshape(-1, 2)


def solve_qp(x, y, th, ref, obs, u_nom, p: QPParams, use_clf=True):
    """Resolve  min ||u - u_nom||^2 + psc*delta^2  com CLF (com folga), CBFs e limites."""
    h, Lgh = cbf_terms(x, y, th, obs, p.l, p.rs)
    rows, rhs = [], []
    if use_clf:
        V, LgV, Vt = clf_terms(x, y, th, ref, p.ctheta)
        rows.append([LgV[0], LgV[1], -1.0]); rhs.append(-p.c * V - Vt)        # LgV u - delta <= -cV - Vt
    for i in range(len(h)):
        rows.append([-Lgh[i, 0], -Lgh[i, 1], 0.0]); rhs.append(p.lam * h[i])  # -Lgh u <= lam h
    for r in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]):
        rows.append(r); rhs.append(p.vmax)                                     # |v|, |w| <= vmax
    H = np.diag([1.0, 1.0, p.psc])
    f = np.array([-u_nom[0], -u_nom[1], 0.0])
    z = qphild(H, f, np.array(rows, float), np.array(rhs, float))
    return z[:2], (float(h.min()) if len(h) else np.inf)
