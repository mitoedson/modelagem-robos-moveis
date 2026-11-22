"""Métricas usadas no relatório."""
import numpy as np
from .controller import wrap


def distance_to_circle(res, R=2.0, t_skip=8.0):
    """Distância (rms, máxima) ao círculo original, descartando o transitório inicial."""
    d = np.abs(np.hypot(res["x"], res["y"]) - R)[res["t"] >= t_skip]
    return float(np.sqrt(np.mean(d ** 2))), float(d.max())


def distance_to_line(res, S=(0.0, 0.0), G=(6.0, 6.0), t_skip=0.0):
    """Distância (rms, máxima) à reta original. O relatório usa a simulação inteira (t_skip = 0)."""
    S, G = np.array(S, float), np.array(G, float)
    u = (G - S) / np.linalg.norm(G - S); n = np.array([-u[1], u[0]])
    d = np.abs((np.c_[res["x"], res["y"]] - S) @ n)[res["t"] >= t_skip]
    return float(np.sqrt(np.mean(d ** 2))), float(d.max())


def tracking_rms(res, t_skip=8.0):
    """Erro rms até o ponto móvel da referência original."""
    e = np.hypot(res["x"] - res["xd0"], res["y"] - res["yd0"])[res["t"] >= t_skip]
    return float(np.sqrt(np.mean(e ** 2)))


def w_jumps(res, thresh=1.0):
    """Saltos bruscos de w (variação > thresh rad/s em uma amostra): indicador de chattering."""
    return int(np.sum(np.abs(np.diff(res["w"])) > thresh))


def saturation_pct(res, vmax=2.0):
    return float(100 * np.mean(np.abs(res["w"]) >= 0.995 * vmax))


def goal_errors(res, G, thG):
    """Erro final de posição [m] e de orientação [rad] (após o último passo de integração)."""
    return (float(np.hypot(res["x_end"] - G[0], res["y_end"] - G[1])),
            float(abs(wrap(res["th_end"] - thG))))


def min_h(res):
    """Menor folga das barreiras; >= 0 significa que nenhum obstáculo foi invadido."""
    return float(res["h"].min())
