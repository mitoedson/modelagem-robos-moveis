# 13. Guia de parâmetros e solução de problemas

## 13.1 Parâmetros do `INIT` (valem para a simulação inteira)

| Variável | Significado | Efeito de aumentar | Valores do estudo |
|---|---|---|---|
| `x0, y0, theta0` | pose inicial do robô (condições iniciais dos integradores) | só muda a partida | ver [08](08-cenarios.md) |
| `xo, yo` | centros dos obstáculos (vetores) | | ver [08](08-cenarios.md) |
| `Ro` | raio físico do obstáculo | **só o desenho**; o controle usa `rs` | 0,1 m |
| `rs` | raio da região de segurança | desvio mais largo e mais cedo | 0,3 m (0,2 m testado) |
| `l` | distância do ponto de controle à frente do eixo | mais autoridade de $w$ na barreira; desvio maior | 0,2 m |
| `lambda` | ganho da CBF | chega mais perto da borda antes de frear; desvia mais tarde | 10 |
| `c` | taxa de decaimento da CLF | retoma mais rápido, mais bruscamente | 0,3 |
| `psc` | peso da folga da CLF | segue mais a referência; **muito grande causa oscilação** | 1000 (círculo); 100 (reta, ponto) |
| `xg, yg, thg` | ponto de chegada e orientação final (só desenho no modelo entregue) | | (5, 5, π/4) |
| `tspan` | tempo de simulação | mais voltas | 60 s |

## 13.2 Parâmetros dentro do bloco

| Variável | Onde | Significado | Valor |
|---|---|---|---|
| `kx, ky, kth` | circular, reta | ganhos do Kanayama | (4,4,6) e (2,10,3) |
| `kp, ka, kb, krot` | ponto | ganhos do controlador polar | 0,8; 2; −0,5; 2 |
| `Lrot` | ponto | distância de $G$ para girar no lugar | 0,005 m |
| `m` | todos | folga além de `rs` | 0,05 m |
| `pw` | todos | meia-largura da deformação | 0,7 rad; 0,8 m; 1,2 m |
| `tolo` | circular | tolerância da regra de lado | 0 |
| `Ld, Lb, Rsen` | ponto | alvo à frente; mistura da orientação final; alcance do sensor | 0,6; 1,5; 3,0 m |
| `wb, hact, side` | todos | viés de lado (desligado com `wb = 0`) | 0; 1,0; 1 |
| `ctheta` | todos | peso do ângulo em $V$ | 1 |
| limites `2` em `bqp` | todos | $|v|, |w| \le 2$ | substituir pelos limites reais do robô |
| `G, thG` / `S, G, Tf` / `R, w_ref_const` | ponto / reta / circular | definição da referência | ver [08](08-cenarios.md) |

## 13.3 Sample time e solver

- **Sample time do bloco = 0,01 s** (Update method = *Discrete*), ou menor. Com passo grande, a CBF perde a garantia entre amostras: nos testes, com $T_s = 0{,}5$ s o $h$ chegou a −0,08; com $T_s = 0{,}05$ s houve mais saturação de $w$.
- `QPhild`: 200 iterações. Subir se houver muitos obstáculos e o controle ficar errático.

## 13.4 Como dimensionar `rs` e `l` para um robô real

Uma estimativa simples: $r_s \ge R_o + r_{robô} + e_{pos} + f$, com

- $R_o$ o raio do obstáculo;
- $r_{robô}$ o maior alcance do corpo (incluindo dispositivos acoplados) a partir do **ponto de controle**;
- $e_{pos}$ o erro de localização;
- $f$ uma folga.

Os valores do estudo (0,2 m e 0,3 m) são de simulação.

## 13.5 Sintomas e ajustes

| Sintoma | Ajuste |
|---|---|
| $w$ oscila entre ±2 | $psc$ menor (1000 ou menos); conferir sample time |
| O robô desvia demais da trajetória | `rs` menor (respeitando o robô real); regra de menor desvio (`tolo = 0`); `m` menor |
| Retomada lenta após o obstáculo | $k_y$ maior (lembrar que a correção lateral é ∝ $v_d$); com a referência deformada, o efeito é menor |
| Picos de $v$ e $w$ na saída do obstáculo | $c$, $k_x$ e $k_y$ menores; limites de $v$ e $w$ menores |
| Traçado em "polígonos" | sample time maior que o necessário; reduzir para 0,01 s |
| O robô para diante do obstáculo | referência deformada ligada? obstáculo sobre o caminho sem deformação ([06](06-simetria-e-escolha-do-lado.md)) |
| Obstáculo ignorado | obstáculos conferidos em `xo`, `yo` e no mesmo modelo que roda |
| Robô não chega ao ponto final | nominal polar (não Kanayama); `Lrot` pequeno; cortes e guardas ([09, 9.7](09-codigo-matlab.md)) |

## 13.6 Alterar o cenário

- **Mais obstáculos:** acrescentar um valor em `xo`, `yo` e `Ro`. Obstáculos muito próximos entre si (deformações se tocam) e passagens menores que $2 r_s$ merecem atenção.
- **Outro círculo:** as constantes `R`, `w_ref_const` e o centro estão no bloco; a regra de lado do círculo supõe centro na origem e sentido anti-horário.
- **Outro ponto de chegada:** `G`, `thG` no bloco **e** `xg, yg, thg` no `INIT` (veja [09, 9.7(c)](09-codigo-matlab.md)).
