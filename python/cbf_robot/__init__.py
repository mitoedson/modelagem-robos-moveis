"""Réplica em Python da lógica CLF-CBF-QP dos blocos MATLAB Function (Simulink).

Módulos:
    qp          -- solver de programação quadrática pelo método de Hildreth (QPhild.m)
    controller  -- CLF, CBFs e montagem/solução do QP
    nominal     -- controles nominais (Kanayama e polar) e vieses de lado
    references  -- referências deformadas (círculo, reta) e alvo reativo (ponto de chegada)
    scenarios   -- simulações completas dos três cenários
    metrics     -- métricas usadas no relatório
"""
from .qp import qphild
from . import controller, nominal, references, scenarios, metrics

__all__ = ["qphild", "controller", "nominal", "references", "scenarios", "metrics"]
