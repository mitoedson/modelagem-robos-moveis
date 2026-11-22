# Réplica em Python

Pacote `cbf_robot`: a mesma lógica CLF-CBF-QP dos blocos MATLAB, para estudar, medir e variar parâmetros sem o Simulink. Detalhes em [`docs/10-simulacao-python.md`](../docs/10-simulacao-python.md).

```bash
pip install -r requirements.txt
python examples/run_circular.py
python examples/run_line.py
python examples/run_goal.py
python examples/make_figures.py      # figuras em ../figures/python e métricas em ../results
pytest -q tests
```

```
cbf_robot/
├── qp.py           # solver de Hildreth (QPhild.m)
├── controller.py   # CLF, CBFs e o QP
├── nominal.py      # Kanayama, polar, vieses de lado
├── references.py   # referências deformadas e alvo reativo
├── scenarios.py    # run_circular, run_line, run_goal
└── metrics.py      # métricas do relatório
```
