"""Gera as figuras (../figures/python) e as métricas (../results) usadas nos documentos.

Uso:  python examples/make_figures.py          (cerca de 1 a 2 minutos; usa todos os núcleos)
"""
import os, sys, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from cbf_robot import scenarios as S, metrics as M
from cbf_robot.controller import QPParams

FIG = os.path.join(HERE, "..", "..", "figures", "python")
RES = os.path.join(HERE, "..", "..", "results")
os.makedirs(FIG, exist_ok=True); os.makedirs(RES, exist_ok=True)
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 200, "savefig.bbox": "tight"})
ANG = np.linspace(0, 2 * np.pi, 300)

OBS_CIRC = S.CircularConfig().obstacles
OBS_CIRC_PSC = ((-1, -1.5), (0, 2.2), (1.5, -1.0), (-1.8, 0.8))      # conjunto usado no estudo do psc
OBS_LINE_PY = ((2, 2), (4, 4.4), (4, 3))                              # conjunto usado nas comparações da reta
OBS_GOAL = ((1.5, 2), (3, 3), (4, 4.6))
POSES = {"(0, 0), θ = 1 rad": (0., 0., 1.), "(0, 0), θ = π (de costas)": (0., 0., np.pi),
         "(5, 0), θ = π/2": (5., 0., np.pi / 2), "(0, 5), θ = −1 rad": (0., 5., -1.)}


def job(name):
    q = lambda **k: QPParams(**k)
    if name == "circ_final":   return S.run_circular()
    if name == "circ_fixed":   return S.run_circular(S.CircularConfig(mode="original", bias="fixed", qp=q(psc=1000.)))
    if name.startswith("psc_"):
        psc = float(name[4:])
        return S.run_circular(S.CircularConfig(obstacles=OBS_CIRC_PSC, mode="original", bias="fixed", qp=q(rs=0.2, psc=psc)))
    if name == "tol_0p1":
        cfg = S.CircularConfig(); cfg.ref.tolo = 0.1; return S.run_circular(cfg)
    if name == "line_bias":    return S.run_line(S.LineConfig(obstacles=OBS_LINE_PY, mode="original", bias="directional"))
    if name == "line_def":     return S.run_line(S.LineConfig(obstacles=OBS_LINE_PY))
    if name == "line_B_bias":  return S.run_line(S.LineConfig(obstacles=((2, 2), (4, 4)), mode="original", bias="directional"))
    if name == "line_B_def":   return S.run_line(S.LineConfig(obstacles=((2, 2), (4, 4))))
    if name == "line_init":    return S.run_line()
    if name == "goal_init":    return S.run_goal(S.GoalConfig(T=40.))
    if name.startswith("pose"):
        k = list(POSES)[int(name[4:])]
        return S.run_goal(S.GoalConfig(G=(3., 4.), start=POSES[k], obstacles=OBS_GOAL, T=40.))
    if name == "far_guard":    return _far(True)
    if name == "far_noguard":  return _far(False)
    raise KeyError(name)


def _far(guard):
    cfg = S.CircularConfig(obstacles=((0.0, 1.0),)); cfg.ref.guard = guard
    return S.run_circular(cfg)


def draw_obs(ax, obs, rs, Ro=0.1, label=True):
    for k, (ox, oy) in enumerate(obs):
        ax.plot(ox + Ro * np.cos(ANG), oy + Ro * np.sin(ANG), "r-", lw=1, label="Obstáculo" if (label and k == 0) else None)
        ax.plot(ox + rs * np.cos(ANG), oy + rs * np.sin(ANG), "r:", lw=1, label="Região de segurança" if (label and k == 0) else None)


def traj(ax, res, obs, rs, ref_xy=None, ref_label="Referência", goal=None, title=None):
    draw_obs(ax, obs, rs)
    if ref_xy is not None: ax.plot(*ref_xy, "g--", lw=1.6, label=ref_label)
    ax.plot(res["x"], res["y"], "b", lw=1.3, label="Robô")
    ax.plot(res["x"][0], res["y"][0], "o", mfc="none", mec="darkorange", ms=7, mew=1.8, label="Inicial")
    if goal is None: ax.plot(res["x"][-1], res["y"][-1], "s", mfc="none", mec="darkorange", ms=7, mew=1.8, label="Final")
    else: ax.plot(*goal, "s", mfc="none", mec="darkorange", ms=9, mew=2, label="Final (chegada)")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)")
    if title: ax.set_title(title, fontsize=9)


def signals(res, path, ylim_h=(-0.5, 8)):
    fig, ax = plt.subplots(3, 2, figsize=(8, 6.2))
    t = res["t"]
    ax[0, 0].plot(t, res["x"], "r", label="Atual"); ax[0, 0].plot(t, res["xd0"], "b", label="Referência"); ax[0, 0].set_ylabel("x (m)"); ax[0, 0].legend(fontsize=7)
    ax[0, 1].plot(t, res["y"], "r", label="Atual"); ax[0, 1].plot(t, res["yd0"], "b", label="Referência"); ax[0, 1].set_ylabel("y (m)")
    wr = lambda a: np.arctan2(np.sin(a), np.cos(a))
    ax[1, 0].plot(t, wr(res["th"]), "r", label="Atual"); ax[1, 0].plot(t, res["thd0"], "b", label="Referência"); ax[1, 0].set_ylabel("θ (rad)")
    ax[1, 1].plot(t, res["v"]); ax[1, 1].set_ylabel("v (m/s)")
    ax[2, 0].plot(t, res["w"]); ax[2, 0].set_ylabel("w (rad/s)")
    ax[2, 1].plot(t, res["h"]); ax[2, 1].axhline(0, color="r", ls="--", lw=1); ax[2, 1].set_ylabel("h"); ax[2, 1].set_ylim(*ylim_h)
    for a in ax.ravel(): a.set_xlabel("t (s)")
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def lgd(fig, ax, n=6):
    h, l = ax.get_legend_handles_labels(); fig.legend(h, l, fontsize=7, loc="lower center", ncol=n, frameon=False)


def main():
    names = (["circ_final", "circ_fixed", "tol_0p1", "line_bias", "line_def", "line_B_bias", "line_B_def", "line_init", "goal_init",
              "far_guard", "far_noguard"] + ["psc_%d" % p for p in (10000, 3000, 1000, 300, 100)] + ["pose%d" % i for i in range(4)])
    with ProcessPoolExecutor() as ex:
        R = dict(zip(names, ex.map(job, names)))
    cc = lambda t: (2 * np.cos(ANG), 2 * np.sin(ANG))

    # 1) círculo: lado fixo x referência deformada
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.7), sharey=True)
    traj(ax[0], R["circ_fixed"], OBS_CIRC, 0.3, cc(0), "Referência (círculo)", title="(a) Lado fixo (viés de lado)")
    traj(ax[1], R["circ_final"], OBS_CIRC, 0.3, cc(0), "Referência (círculo)", title="(b) Referência interna deformada")
    ax[0].set_ylabel("y (m)"); lgd(fig, ax[0]); fig.tight_layout(rect=(0, 0.07, 1, 1)); fig.savefig(f"{FIG}/circular_lado_fixo_vs_deformada.png"); plt.close(fig)
    # 2) tolerância da regra de lado
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.7), sharey=True)
    traj(ax[0], R["tol_0p1"], OBS_CIRC, 0.3, cc(0), "Referência (círculo)", title="(a) tolerância 0,1 m")
    traj(ax[1], R["circ_final"], OBS_CIRC, 0.3, cc(0), "Referência (círculo)", title="(b) tolerância 0")
    ax[0].set_ylabel("y (m)"); lgd(fig, ax[0]); fig.tight_layout(rect=(0, 0.07, 1, 1)); fig.savefig(f"{FIG}/circular_tolerancia_regra_lado.png"); plt.close(fig)
    # 3) psc e chattering
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 3.6), sharex=True)
    for a, n, t in ((ax[0], "psc_10000", "psc = 10000"), (ax[1], "psc_1000", "psc = 1000")):
        a.plot(R[n]["t"], R[n]["w"], lw=0.8); a.set_ylim(-2.3, 2.3); a.set_ylabel("w (rad/s)"); a.set_title(t, fontsize=9, loc="left")
    ax[1].set_xlabel("t (s)"); fig.tight_layout(); fig.savefig(f"{FIG}/psc_chattering.png"); plt.close(fig)
    # 4) reta
    ln = ([0, 6], [0, 6])
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.5), sharey=True)
    traj(ax[0], R["line_bias"], OBS_LINE_PY, 0.3, ln, "Reta de referência", title="(a) Viés de lado")
    traj(ax[1], R["line_def"], OBS_LINE_PY, 0.3, ln, "Reta de referência", title="(b) Referência deformada")
    for a in ax: a.set_xlim(-0.5, 6.5); a.set_ylim(-0.5, 6.5)
    ax[0].set_ylabel("y (m)"); lgd(fig, ax[0]); fig.tight_layout(rect=(0, 0.07, 1, 1)); fig.savefig(f"{FIG}/reta_vies_vs_deformada.png"); plt.close(fig)
    # 5) ponto de chegada, quatro poses
    fig, ax = plt.subplots(figsize=(5.8, 4.8)); draw_obs(ax, OBS_GOAL, 0.3)
    cols = ["tab:blue", "tab:purple", "tab:green", "tab:brown"]
    for j, (k, st) in enumerate(POSES.items()):
        r = R["pose%d" % j]; ax.plot(r["x"], r["y"], color=cols[j], lw=1.3, label="Partida " + k); ax.plot(st[0], st[1], "o", mfc="none", mec=cols[j], ms=6, mew=1.5)
        ax.annotate("", xy=(st[0] + 0.45 * np.cos(st[2]), st[1] + 0.45 * np.sin(st[2])), xytext=(st[0], st[1]), arrowprops=dict(arrowstyle="->", color=cols[j], lw=1.5))
    ax.plot(3, 4, "s", mfc="none", mec="darkorange", ms=10, mew=2, label="Final (chegada)")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, frameon=False)
    fig.tight_layout(); fig.savefig(f"{FIG}/ponto_poses_iniciais.png"); plt.close(fig)
    # 6) equivalentes aos gráficos do Simulink (mesmos INIT)
    fig, ax = plt.subplots(figsize=(5.2, 4.6)); traj(ax, R["circ_final"], OBS_CIRC, 0.3, cc(0), "Referência (círculo original)"); ax.legend(fontsize=6.5, loc="upper right"); ax.set_ylabel("y (m)"); fig.tight_layout(); fig.savefig(f"{FIG}/equiv_simulink_circular_trajetoria.png"); plt.close(fig)
    signals(R["circ_final"], f"{FIG}/equiv_simulink_circular_sinais.png")
    O = S.LineConfig().obstacles
    fig, ax = plt.subplots(figsize=(5.2, 4.6)); traj(ax, R["line_init"], O, 0.3, ln, "Referência (reta original)"); ax.legend(fontsize=6.5, loc="lower right"); ax.set_ylabel("y (m)"); fig.tight_layout(); fig.savefig(f"{FIG}/equiv_simulink_reta_trajetoria.png"); plt.close(fig)
    signals(R["line_init"], f"{FIG}/equiv_simulink_reta_sinais.png")
    g = S.GoalConfig()
    fig, ax = plt.subplots(figsize=(5.2, 4.6)); traj(ax, R["goal_init"], g.obstacles, 0.3, None, goal=g.G); ax.legend(fontsize=6.5, loc="lower right"); ax.set_ylabel("y (m)"); fig.tight_layout(); fig.savefig(f"{FIG}/equiv_simulink_ponto_trajetoria.png"); plt.close(fig)
    signals(R["goal_init"], f"{FIG}/equiv_simulink_ponto_sinais.png", ylim_h=(-0.5, 4))

    # ---------------- métricas
    m = {}
    f3 = lambda v: [round(x, 3) for x in v]
    m["circular"] = {"deformada": f3(M.distance_to_circle(R["circ_final"])), "lado_fixo": f3(M.distance_to_circle(R["circ_fixed"])),
                     "tolerancia_0p1": f3(M.distance_to_circle(R["tol_0p1"])),
                     "min_h_deformada": round(M.min_h(R["circ_final"]), 3), "min_h_lado_fixo": round(M.min_h(R["circ_fixed"]), 3),
                     "obstaculo_longe": {"sem_guarda": f3(M.distance_to_circle(R["far_noguard"])), "com_guarda": f3(M.distance_to_circle(R["far_guard"]))}}
    m["psc"] = {p: {"saltos_w": M.w_jumps(R["psc_" + p]), "w_saturado_pct": round(M.saturation_pct(R["psc_" + p]), 1),
                    "v_negativo_pct": round(100 * float(np.mean(R["psc_" + p]["v"] < -0.05)), 1), "erro_rms_ponto_movel": round(M.tracking_rms(R["psc_" + p]), 3)}
                for p in ("10000", "3000", "1000", "300", "100")}
    m["reta"] = {"A_vies": f3(M.distance_to_line(R["line_bias"])), "A_deformada": f3(M.distance_to_line(R["line_def"])),
                 "B_vies": f3(M.distance_to_line(R["line_B_bias"])), "B_deformada": f3(M.distance_to_line(R["line_B_def"])),
                 "min_h_A_deformada": round(M.min_h(R["line_def"]), 3)}
    m["ponto_poses_(3,4)"] = {}
    for j, k in enumerate(POSES):
        e = M.goal_errors(R["pose%d" % j], (3., 4.), np.pi / 4)
        m["ponto_poses_(3,4)"][k] = {"erro_pos_cm": round(100 * e[0], 2), "erro_ori_rad": round(e[1], 4), "saltos_w": M.w_jumps(R["pose%d" % j]), "min_h": round(M.min_h(R["pose%d" % j]), 3)}
    e = M.goal_errors(R["goal_init"], g.G, g.thG)
    m["ponto_INIT_(5,5)"] = {"erro_pos_cm": round(100 * e[0], 2), "erro_ori_rad": round(e[1], 4), "min_h": round(M.min_h(R["goal_init"]), 3)}
    json.dump(m, open(os.path.join(RES, "metricas.json"), "w"), indent=2, ensure_ascii=False)
    print(json.dumps(m, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
