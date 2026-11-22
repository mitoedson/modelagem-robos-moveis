# Variante: ponto de chegada lido do INIT

`SIMU_DIF_CBF_ponto_sensor_param.slx` difere do modelo entregue (`../SIMU_DIF_CBF_ponto_sensor.slx`) em três pontos:

1. **Parâmetros novos** `xg`, `yg`, `thg` (declarados no bloco, como `xo`, `yo`): o ponto de chegada passa a ser definido **só no `INIT`** (que já define essas variáveis).
2. **Corte de obstáculos atrás do alvo:** `so < min(Rsen, Lg + 0.3)`.
3. Assinatura do bloco: `function out = fcn(u, xo, yo, xg, yg, thg, l, rs, psc, c, lambda)`.

> **Não foi validada no Simulink.** O modelo foi gerado editando o XML do arquivo. Se o Simulink acusar erro em `xg`, `yg` ou `thg`, use o modelo entregue e mantenha `G`/`thG` (no bloco) iguais a `xg, yg, thg` (no `INIT`).

Uso: no `INIT_DIF_CBF_ponto_sensor.m`, troque o nome em `sim(...)` para `'SIMU_DIF_CBF_ponto_sensor_param'`.
