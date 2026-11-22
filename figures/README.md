# Figuras

- [`python/`](python): geradas por `python/examples/make_figures.py` (réplica Python). `equiv_simulink_*` usam as mesmas configurações dos `INIT` do Simulink.
- [`simulink/`](simulink): capturas de tela dos gráficos do Simulink (tema escuro do MATLAB).

| Arquivo (`simulink/`) | Conteúdo |
|---|---|
| `circular_trajetoria.jpg`, `circular_sinais.jpg` | círculo, 4 obstáculos, tolerância 0 (versão final) |
| `circular_tolerancia0p1_trajetoria.jpg`, `..._sinais.jpg` | círculo com tolerância 0,1 m (obstáculo da direita contornado por fora) |
| `reta_trajetoria.jpg`, `reta_sinais.jpg` | reta com 3 obstáculos |
| `ponto_5_5_trajetoria.jpg`, `ponto_5_5_sinais.jpg` | ponto de chegada (5, 5), pose inicial (0, 0, 0) |
| `ponto_6_4_trajetoria.jpg`, `ponto_6_4_sinais.jpg` | ponto de chegada (6, 4); `INIT` não incluído, obstáculos estimados da figura |

Nos gráficos de sinais, de cima para baixo e da esquerda para a direita: $x$, $y$, $\theta$, $v$, $w$ e $h$.
