"""Ponto de chegada com os dados de INIT_DIF_CBF_ponto_sensor.m (alvo (5, 5), pose inicial (0, 0, 0))."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cbf_robot import scenarios as S, metrics as M

cfg = S.GoalConfig()
r = S.run_goal(cfg)
pos, ori = M.goal_errors(r, cfg.G, cfg.thG)
print("Erro final: posição = %.3f m, orientação = %.4f rad" % (pos, ori))
print("h mínimo:", round(M.min_h(r), 3))
