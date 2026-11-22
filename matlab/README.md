# MATLAB e Simulink

Três cenários, cada um com **modelo Simulink** + **arquivo de inicialização (`INIT`)** + **cópia legível do código do bloco**.

| Pasta | Cenário | Modelo | INIT | Código legível |
|---|---|---|---|---|
| [`circular/`](circular) | trajetória circular | `SIMU_DIF_CBF_v2_refcirc.slx` | `INIT_DIF_CBF_circular.m` | `fcn_circular_bloco.m` |
| [`reta/`](reta) | linha reta de (0,0) a (6,6) | `SIMU_DIF_CBF_reta_refdef.slx` | `INIT_DIF_CBF_reta_refdef.m` | `fcn_reta_bloco.m` |
| [`ponto/`](ponto) | ponto de chegada (sem trajetória) | `SIMU_DIF_CBF_ponto_sensor.slx` | `INIT_DIF_CBF_ponto_sensor.m` | `fcn_ponto_bloco.m` |
| [`common/`](common) | compartilhados | | | `QPhild.m`, `LIE_DIF_CBF.m` |

## Como rodar

1. No MATLAB, na pasta `matlab/`: `setup`.
2. `cd circular` (ou `reta`, `ponto`).
3. `bdclose all`.
4. Abra o `INIT_*.m` **no editor** e clique em **Run**. Ele define os parâmetros, chama `sim(...)` e desenha as figuras.
5. Confira o **sample time = 0,01 s** do bloco *MATLAB Function*.

> Não use o botão Play do Simulink sem rodar antes o `INIT`: faltarão `x0`, `c`, `xo`, etc.

## Requisitos

MATLAB com Simulink; `wrapToPi` (Mapping Toolbox) nos gráficos; *Symbolic Math Toolbox* só para `common/LIE_DIF_CBF.m`.

## Leia também

- [Estudo do código MATLAB](../docs/09-codigo-matlab.md): anatomia do bloco, interface, erros comuns.
- **Pontos de atenção e patches propostos** (guarda do círculo, corte do ponto de chegada, ponto de chegada em dois lugares): [`docs/09-codigo-matlab.md`, seção 9.7](../docs/09-codigo-matlab.md). Os `.slx` deste repositório **são os entregues, sem alterações**.
- Os `*_bloco.m` são cópias para **leitura**; o código executado está dentro dos `.slx`. Comentários de versões antigas foram removidos.
- `ponto/variantes/`: modelo em que `xg`, `yg`, `thg` são lidos do `INIT` (como `xo`, `yo`) e com o corte de obstáculos atrás do alvo. **Não validado no Simulink.** Para usá-lo, troque `sim('SIMU_DIF_CBF_ponto_sensor', tspan)` por `sim('SIMU_DIF_CBF_ponto_sensor_param', tspan)` no `INIT`.
