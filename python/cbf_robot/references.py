"""Referências deformadas em torno dos obstáculos e alvo reativo do ponto de chegada.

Ideia comum: o controle persegue uma referência deslocada lateralmente por um 'morro' de
cosseno elevado,  b(u) = 0.5 (1 + cos(pi u)),  |u| < 1, centrado em cada obstáculo. O deslocamento
deixa a folga  rs + m  do obstáculo, pelo lado de MENOR desvio. A referência exibida nos gráficos
continua sendo a original.
"""
from dataclasses import dataclass
import numpy as np
from .controller import Ref, wrap


# ---------------------------------------------------------------- círculo
@dataclass
class CircleRef:
    R: float = 2.0
    w: float = 0.25          # velocidade angular da referência [rad/s]
    m: float = 0.05          # folga além de rs [m]
    pw: float = 0.7          # meia-largura angular da deformação [rad]
    tolo: float = 0.0        # ro <= R+tolo: passa por fora; senão, por dentro
    guard: bool = True       # ignora obstáculos com |ro-R| >= rs+m (não bloqueiam o caminho)

    def original(self, t):
        phi = self.w * t
        return (self.R * np.cos(phi), self.R * np.sin(phi),
                np.arctan2(self.R * self.w * np.cos(phi), -self.R * self.w * np.sin(phi)))

    def as_ref(self, t):
        xd, yd, thd = self.original(t)
        phi = self.w * t
        return Ref(xd, yd, -self.R * self.w * np.sin(phi), self.R * self.w * np.cos(phi), thd, self.w)

    def deformed(self, t, obs, rs):
        phi, R = self.w * t, self.R
        Rr = R; d1 = 0.0; d2 = 0.0
        for ox, oy in obs:
            ro, ph = np.hypot(ox, oy), np.arctan2(oy, ox)
            if self.guard and abs(ro - R) >= rs + self.m:
                continue
            delta = (ro + rs + self.m - R) if ro <= R + self.tolo else (ro - rs - self.m - R)
            u = (phi - ph + np.pi) % (2 * np.pi) - np.pi
            if abs(u) < self.pw:
                k = np.pi / self.pw
                Rr += delta * 0.5 * (1 + np.cos(k * u))
                d1 += -delta * 0.5 * np.sin(k * u) * k
                d2 += -delta * 0.5 * np.cos(k * u) * k ** 2
        xd, yd = Rr * np.cos(phi), Rr * np.sin(phi)
        xdd = self.w * (d1 * np.cos(phi) - Rr * np.sin(phi))
        ydd = self.w * (d1 * np.sin(phi) + Rr * np.cos(phi))
        thd = np.arctan2(ydd, xdd)
        thdd = self.w * (1 + (d1 ** 2 - Rr * d2) / (Rr ** 2 + d1 ** 2))
        return Ref(xd, yd, xdd, ydd, thd, thdd)


# ---------------------------------------------------------------- reta
@dataclass
class LineRef:
    S: tuple = (0.0, 0.0)
    G: tuple = (6.0, 6.0)
    Tf: float = 60.0
    m: float = 0.05
    pw: float = 0.8          # meia-largura da deformação ao longo da reta [m]
    tie_right: bool = True   # obstáculo exatamente sobre a reta: passa pela direita

    def __post_init__(self):
        S, G = np.array(self.S, float), np.array(self.G, float)
        self.L = np.linalg.norm(G - S)
        self.u = (G - S) / self.L
        self.n = np.array([-self.u[1], self.u[0]])
        self.vd = self.L / self.Tf

    def original(self, t):
        p = np.array(self.S) + min(self.vd * t, self.L) * self.u
        return p[0], p[1], np.arctan2(self.u[1], self.u[0])

    def as_ref(self, t):
        xd, yd, thd = self.original(t)
        return Ref(xd, yd, self.vd * self.u[0], self.vd * self.u[1], thd, 0.0)

    def deformed(self, t, obs, rs):
        S = np.array(self.S, float)
        s = min(self.vd * t, self.L); cm = rs + self.m
        l0 = l1 = l2 = 0.0
        for o in obs:
            so = (np.array(o) - S) @ self.u
            dl = (np.array(o) - S) @ self.n
            if abs(dl) >= cm:
                continue
            ell = (-cm if self.tie_right else cm) if abs(dl) < 1e-3 else dl - np.sign(dl) * cm
            sg = (s - so) / self.pw
            if abs(sg) < 1:
                k = np.pi / self.pw
                l0 += ell * 0.5 * (1 + np.cos(np.pi * sg))
                l1 += -ell * 0.5 * np.sin(np.pi * sg) * k
                l2 += -ell * 0.5 * np.cos(np.pi * sg) * k ** 2
        pos = S + s * self.u + l0 * self.n
        vel = self.vd * (self.u + l1 * self.n)
        return Ref(pos[0], pos[1], vel[0], vel[1], np.arctan2(vel[1], vel[0]), self.vd * l2 / (1 + l1 ** 2))


# ---------------------------------------------------------------- ponto de chegada
def goal_target(x, y, th, G, thG, obs, rs, m=0.05, pw=1.2, Ld=0.6, Lb=1.5, Rsen=3.0, cutoff=True):
    """Alvo reativo (sem trajetória): a cada instante, um ponto a Ld à frente do robô, na direção
    de G, deslocado lateralmente quando um obstáculo bloqueia o caminho direto.

    Retorna (alvo, orientação de referência, fator de mistura da orientação final, distância a G).
    ``cutoff``: ignora obstáculos além de G (evita que um obstáculo atrás do alvo o desloque).
    """
    p = np.array([x, y], float); G = np.array(G, float)
    g = G - p; Lg = np.linalg.norm(g)
    ur = g / Lg if Lg > 1e-6 else np.array([np.cos(th), np.sin(th)])
    nr = np.array([-ur[1], ur[0]]); ang = np.arctan2(ur[1], ur[0])
    st = min(Lg, Ld); cm = rs + m
    l0 = l1 = 0.0
    for o in obs:
        so = (np.array(o) - p) @ ur
        dl = (np.array(o) - p) @ nr
        upper = min(Rsen, Lg + 0.3) if cutoff else Rsen
        if so < -0.3 or so > upper or abs(dl) >= cm:
            continue
        ell = -cm if abs(dl) < 1e-3 else dl - np.sign(dl) * cm
        sg = (st - so) / pw
        if abs(sg) < 1:
            l0 += ell * 0.5 * (1 + np.cos(np.pi * sg))
            l1 += -ell * 0.5 * np.sin(np.pi * sg) * np.pi / pw
    tgt = p + st * ur + l0 * nr
    tht = ang + np.arctan(l1)
    lamb = min(max(1 - Lg / Lb, 0.0), 1.0)
    thref = tht + lamb * wrap(thG - tht)
    return tgt, thref, lamb, Lg
