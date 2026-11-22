# 10. Simulação em Python (réplica da lógica do bloco)

## 10.1 Para que serve

O pacote [`python/cbf_robot`](../python/cbf_robot) reproduz, linha a linha, a lógica dos blocos MATLAB Function: mesma CLF, mesmas CBFs, mesmo QP (Hildreth), mesmos controles nominais e mesmas referências deformadas. Serve para:

- **estudar** o algoritmo sem o Simulink (cada simulação de 60 s leva menos de 1 s);
- **medir** o que no Simulink só se vê em gráficos (erro rms, saltos de $w$, folga $h$);
- **testar variações** com facilidade (`dataclass` de configuração por cenário);
- **reproduzir** as tabelas do relatório.

> Os números quantitativos do relatório vêm desta réplica. Os gráficos do Simulink serviram para verificação qualitativa dos três cenários.

## 10.2 Estrutura do pacote

| Módulo | Conteúdo | Equivale a |
|---|---|---|
| `qp.py` | `qphild` | `QPhild.m` |
| `controller.py` | `QPParams`, `clf_terms`, `cbf_terms`, `solve_qp`, `Ref` | CLF, CBFs e montagem de `Aqp`, `bqp` no bloco |
| `nominal.py` | `kanayama`, `polar`, `fixed_side_bias`, `directional_bias` | seção "controle nominal" e `wbias` |
| `references.py` | `CircleRef`, `LineRef`, `goal_target` | seção "referência" dos três blocos |
| `scenarios.py` | `CircularConfig/run_circular`, `LineConfig/run_line`, `GoalConfig/run_goal` | os três modelos `.slx` + `INIT` |
| `metrics.py` | `distance_to_circle`, `distance_to_line`, `tracking_rms`, `w_jumps`, `goal_errors`, `min_h` | métricas do relatório |

As configurações padrão (`CircularConfig()`, `LineConfig()`, `GoalConfig()`) usam os mesmos obstáculos, pose inicial e parâmetros dos `INIT` da pasta `matlab/`.

## 10.3 Instalação e uso

```bash
cd python
python -m venv .venv && source .venv/bin/activate      # opcional
pip install -r requirements.txt

python examples/run_circular.py     # distância ao círculo, h mínimo, saltos de w
python examples/run_line.py
python examples/run_goal.py         # erro final de posição e orientação
python examples/make_figures.py     # gera ../figures/python/*.png e ../results/metricas.json
pytest -q tests                     # 7 testes
```

Exemplo de uso como biblioteca:

```python
from cbf_robot import scenarios as S, metrics as M
from cbf_robot.controller import QPParams

# círculo com lado fixo (viés) em vez da referência deformada
r = S.run_circular(S.CircularConfig(mode="original", bias="fixed", qp=QPParams(psc=1000.)))
print(M.distance_to_circle(r), M.min_h(r), M.w_jumps(r))

# ponto de chegada (3, 4) a partir de uma pose de costas
r = S.run_goal(S.GoalConfig(G=(3., 4.), start=(0., 0., 3.1416), T=40.))
```

Cada `run_*` devolve um dicionário com `t, x, y, th, v, w, h` (sinais do robô), `xd0, yd0, thd0` (referência **exibida**, original) e `xr, yr` (referência interna perseguida).

## 10.4 Validação contra o relatório

Os mesmos números aparecem em [`results/metricas.json`](../results/metricas.json), gerado por `make_figures.py`:

| Experimento | Relatório | Pacote Python |
|---|---|---|
| Círculo, referência deformada, 4 obstáculos: distância ao círculo (rms / máx.) | 0,126 / 0,333 m | 0,126 / 0,333 m |
| Círculo, lado fixo | 0,182 / 0,640 m | 0,182 / 0,640 m |
| $psc = 10000$ → $1000$: saltos de $w$ | 520 → 12 | 520 → 12 |
| Reta, cenário A, deformada / viés: rms / máx. | 0,095 / 0,350 e 0,097 / 0,377 m | 0,095 / 0,350 e 0,097 / 0,377 m |
| Ponto (3, 4): erro final | 0,5 cm; 0,00 rad | 0,49 a 0,50 cm; 0,0000 rad |

Observação sobre métricas: no círculo, descartam-se os primeiros 8 s (transitório); na reta, usa-se a simulação inteira (como no relatório). O erro rms até o ponto móvel difere até 0,003 m do relatório (convenção de alinhamento das amostras).

## 10.5 Comparação com os gráficos do Simulink

As figuras `equiv_simulink_*` em [`figures/python`](../figures/python) usam as mesmas configurações dos `INIT` e reproduzem o comportamento das capturas do Simulink em [`figures/simulink`](../figures/simulink): lados do desvio, chegada em ≈ 18 s no ponto (5, 5), orientação convergindo para $\pi/4$ e $h$ nunca negativo.

## 10.6 Diferenças em relação ao Simulink

| Aspecto | Python | Simulink |
|---|---|---|
| Integração | Euler, $T_s = 0{,}01$ s para planta e controle | solver do modelo; controle discreto de 0,01 s |
| Solver do QP | `qphild` (mesmo algoritmo) | `QPhild.m` via `coder.extrinsic` |
| Modelo do robô | cinemático | cinemático |
| Ruído, atraso, dinâmica | não | não |

## 10.7 Experimentos do histórico que não estão no pacote

Alguns experimentos do estudo usaram *scripts* descartáveis e **não foram portados** para o pacote; seus resultados estão descritos em [12](12-historico-de-melhorias.md) e [06](06-simetria-e-escolha-do-lado.md): busca aleatória de parâmetros; viés por posição radial; varredura de $k_y$ na reta; varredura de $L_{rot}$; efeito do período $T_s$ grande. Eles podem ser refeitos alterando as configurações (`CircularConfig`, `LineConfig`, `GoalConfig`).
