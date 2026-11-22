"""Simulações completas dos três cenários (modelo cinemático, integração de Euler, Ts = 0,01 s).

Cada ``run_*`` devolve um dicionário de séries temporais com os mesmos sinais que o bloco do
Simulink envia ao workspace: t, x, y, th, v, w, h, e a referência EXIBIDA (original):
xd0, yd0, thd0. Além disso, ``xr``, ``yr`` trazem a referência interna efetivamente perseguida.
"""
from dataclasses import dataclass, field
import numpy as np
from .controller import QPParams, Ref, solve_qp
from . import nominal
from .references import CircleRef, LineRef, goal_target


def _euler(x, y, th, u, Ts):
    return x + Ts * u[0] * np.cos(th), y + Ts * u[0] * np.sin(th), th + Ts * u[1]


def _pack(rows, extra=None):
    A = np.array(rows, float)
    out = dict(t=A[:, 0], x=A[:, 1], y=A[:, 2], th=A[:, 3], v=A[:, 4], w=A[:, 5], h=A[:, 6],
               xd0=A[:, 7], yd0=A[:, 8], thd0=A[:, 9], xr=A[:, 10], yr=A[:, 11])
    if extra:
        out.update(extra)
    return out


# ------------------------------------------------------------------ círculo
@dataclass
class CircularConfig:
    obstacles: tuple = ((-1.0, -1.5), (0.0, 2.2), (1.75, -1.0), (-2.0, 0.8))   # = INIT_DIF_CBF_circular.m
    start: tuple = (1.0, 0.0, 0.0)
    qp: QPParams = field(default_factory=lambda: QPParams(psc=1000.0))
    ref: CircleRef = field(default_factory=CircleRef)
    kx: float = 4.0
    ky: float = 4.0
    kth: float = 6.0
    mode: str = "deformed"      # 'deformed': referência deformada | 'original': círculo original
    bias: str = "none"          # 'none' | 'fixed' (viés de lado fixo, só faz sentido com mode='original')
    wb: float = 0.5
    side: int = 1
    Ts: float = 0.01
    T: float = 60.0


def run_circular(cfg: CircularConfig = CircularConfig()):
    x, y, th = cfg.start
    rows = []
    for i in range(int(round(cfg.T / cfg.Ts))):
        t = i * cfg.Ts
        ref = cfg.ref.deformed(t, cfg.obstacles, cfg.qp.rs) if cfg.mode == "deformed" else cfg.ref.as_ref(t)
        wb = (nominal.fixed_side_bias(x, y, th, cfg.obstacles, cfg.qp.l, cfg.qp.rs, cfg.side, cfg.wb)
              if cfg.bias == "fixed" else 0.0)
        u_nom = nominal.kanayama(x, y, th, ref, cfg.kx, cfg.ky, cfg.kth, wb)
        u, h = solve_qp(x, y, th, ref, cfg.obstacles, u_nom, cfg.qp)
        xd0, yd0, thd0 = cfg.ref.original(t)
        rows.append((t, x, y, th, u[0], u[1], h, xd0, yd0, thd0, ref.xd, ref.yd))
        x, y, th = _euler(x, y, th, u, cfg.Ts)
    return _pack(rows)


# ------------------------------------------------------------------ reta
@dataclass
class LineConfig:
    obstacles: tuple = ((2.0, 2.0), (4.2, 4.4), (3.4, 3.2))                      # = INIT_DIF_CBF_reta_refdef.m
    start: tuple = (0.0, 0.0, 1.0)
    qp: QPParams = field(default_factory=lambda: QPParams(psc=100.0))
    ref: LineRef = field(default_factory=LineRef)
    kx: float = 2.0
    ky: float = 10.0
    kth: float = 3.0
    mode: str = "deformed"      # 'deformed' | 'original'
    bias: str = "none"          # 'none' | 'directional'
    wb: float = 1.0
    side: int = 1
    Ts: float = 0.01
    T: float = 60.0


def run_line(cfg: LineConfig = LineConfig()):
    x, y, th = cfg.start
    rows = []
    for i in range(int(round(cfg.T / cfg.Ts))):
        t = i * cfg.Ts
        ref = cfg.ref.deformed(t, cfg.obstacles, cfg.qp.rs) if cfg.mode == "deformed" else cfg.ref.as_ref(t)
        wb = (nominal.directional_bias(x, y, th, cfg.obstacles, cfg.qp.l, cfg.qp.rs, cfg.side, cfg.wb)
              if cfg.bias == "directional" else 0.0)
        u_nom = nominal.kanayama(x, y, th, ref, cfg.kx, cfg.ky, cfg.kth, wb)
        u, h = solve_qp(x, y, th, ref, cfg.obstacles, u_nom, cfg.qp)
        xd0, yd0, thd0 = cfg.ref.original(t)
        rows.append((t, x, y, th, u[0], u[1], h, xd0, yd0, thd0, ref.xd, ref.yd))
        x, y, th = _euler(x, y, th, u, cfg.Ts)
    return _pack(rows)


# ------------------------------------------------------------------ ponto de chegada
@dataclass
class GoalConfig:
    G: tuple = (5.0, 5.0)                                                         # = xg, yg do INIT_DIF_CBF_ponto_sensor.m
    thG: float = np.pi / 4
    start: tuple = (0.0, 0.0, 0.0)
    obstacles: tuple = ((1.67, 1.25), (2.3, 2.6), (4.23, 4.25))
    qp: QPParams = field(default_factory=lambda: QPParams(psc=100.0))
    m: float = 0.05
    pw: float = 1.2
    Ld: float = 0.6
    Lb: float = 1.5
    Rsen: float = 3.0
    cutoff: bool = True          # o .slx entregue NÃO tem este corte (ver docs/09-codigo-matlab.md)
    kp: float = 0.8
    ka: float = 2.0
    kb: float = -0.5
    krot: float = 2.0
    Lrot: float = 0.005
    Ts: float = 0.01
    T: float = 60.0


def run_goal(cfg: GoalConfig = GoalConfig()):
    x, y, th = cfg.start
    rows = []
    for i in range(int(round(cfg.T / cfg.Ts))):
        t = i * cfg.Ts
        tgt, thref, lamb, Lg = goal_target(x, y, th, cfg.G, cfg.thG, cfg.obstacles, cfg.qp.rs,
                                           cfg.m, cfg.pw, cfg.Ld, cfg.Lb, cfg.Rsen, cfg.cutoff)
        ref = Ref(tgt[0], tgt[1], 0.0, 0.0, thref, 0.0)
        u_nom = nominal.polar(x, y, th, tgt, thref, lamb, Lg, cfg.thG, cfg.kp, cfg.ka, cfg.kb, cfg.krot, cfg.Lrot)
        u, h = solve_qp(x, y, th, ref, cfg.obstacles, u_nom, cfg.qp)
        rows.append((t, x, y, th, u[0], u[1], h, cfg.G[0], cfg.G[1], cfg.thG, tgt[0], tgt[1]))
        x, y, th = _euler(x, y, th, u, cfg.Ts)
    out = _pack(rows)
    out.update(x_end=x, y_end=y, th_end=th)
    return out
