"""Controles nominais e vieses de lado."""
import numpy as np
from .controller import pose_errors, wrap


def kanayama(x, y, th, ref, kx, ky, kth, wbias=0.0):
    """Seguimento de trajetória (Kanayama et al., 1990).

    v_nom = vd cos(e_th) + kx ex
    w_nom = th_d' + vd (ky ey + kth sin(e_th)) + wbias
    Com vd = 0 (referência fixa) os termos laterais desaparecem: use ``polar``.
    """
    ex, ey, et = pose_errors(x, y, th, ref)
    vd = np.hypot(ref.xd_dot, ref.yd_dot)
    return np.array([vd * np.cos(et) + kx * ex,
                     ref.thd_dot + vd * (ky * ey + kth * np.sin(et)) + wbias])


def polar(x, y, th, tgt, thref, lamb, Lg, thG, kp=0.8, ka=2.0, kb=-0.5, krot=2.0, Lrot=0.005):
    """Regulação de pose (controlador polar), sem ré, com giro no lugar na chegada.

    rho: distância ao alvo; alpha: ângulo até ele; beta: erro de orientação restante.
    Estabilidade: kp > 0, kb < 0, ka > kp.
    """
    rho = np.hypot(tgt[0] - x, tgt[1] - y)
    al = wrap(np.arctan2(tgt[1] - y, tgt[0] - x) - th)
    be = wrap(thref - th - al)
    v = kp * rho * max(np.cos(al), 0.0)
    w = ka * al + kb * lamb * be
    if Lg < Lrot:                          # chegou: gira no lugar até a orientação final
        v, w = 0.0, krot * wrap(thG - th)
    return np.array([v, w])


def fixed_side_bias(x, y, th, obs, l, rs, side=1, wb=0.5, hact=1.0):
    """Viés de lado fixo (versão inicial): empurra w para o mesmo lado perto de qualquer
    obstáculo à frente. Quebra a simetria, mas não escolhe o lado de menor desvio."""
    px, py = x + l * np.cos(th), y + l * np.sin(th)
    wbias = 0.0
    for ox, oy in obs:
        dx, dy = px - ox, py - oy
        h = dx ** 2 + dy ** 2 - rs ** 2
        dist = max(np.hypot(dx, dy), 1e-6)
        a = max(0.0, min(1.0, 1 - h / hact))
        ahead = max(0.0, (-dx * np.cos(th) - dy * np.sin(th)) / dist)
        wbias += side * a * ahead
    return wb * max(-1.0, min(1.0, wbias))


def directional_bias(x, y, th, obs, l, rs, side=1, wb=1.0, hact=1.0, eps=0.25):
    """Viés condicionado (reta): só age com o obstáculo quase à frente e empurra para o
    lado oposto a ele; ``side`` só desempata quando o obstáculo está no centro (|comp| < eps)."""
    px, py = x + l * np.cos(th), y + l * np.sin(th)
    wbias = 0.0
    for ox, oy in obs:
        dx, dy = px - ox, py - oy
        h = dx ** 2 + dy ** 2 - rs ** 2
        dist = max(np.hypot(dx, dy), 1e-6)
        a = max(0.0, min(1.0, 1 - h / hact))
        ahead = max(0.0, (-dx * np.cos(th) - dy * np.sin(th)) / dist)
        comp = dx * np.sin(th) - dy * np.cos(th)          # > 0: obstáculo à esquerda
        gate = max(0.0, 1 - abs(comp) / (rs + 0.2))
        s = side if abs(comp) < eps else -np.sign(comp)
        wbias += s * a * ahead * gate
    return wb * max(-1.0, min(1.0, wbias))
