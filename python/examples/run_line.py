"""Cenário de reta com os dados de INIT_DIF_CBF_reta_refdef.m."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cbf_robot import scenarios as S, metrics as M

r = S.run_line()
print("Distância à reta (rms, máx) [m]:", [round(v, 3) for v in M.distance_to_line(r)])
print("h mínimo:", round(M.min_h(r), 3), "| saltos de w:", M.w_jumps(r))
