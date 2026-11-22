# 2. Função de Lyapunov de controle (CLF) e seguimento

## 2.1 Ideia

Uma função $V(q, t) \ge 0$ é uma *função de Lyapunov de controle* se, para todo estado, existe uma entrada que a faz decrescer (Sontag, 1989). Impor um decaimento exponencial

$$
\dot V \le -c\,V
$$

como **restrição linear em $u$** transforma o seguimento de referência em uma restrição de um QP ([04-qp-e-solver.md](04-qp-e-solver.md)).

## 2.2 A função usada

$$
V = \tfrac12\,(e_x^2 + e_y^2) + c_\theta\,(1 - \cos e_\theta)
$$

O primeiro termo é metade do quadrado da distância à referência ($e_x^2 + e_y^2 = \|p_d - p\|^2$, invariante à rotação). O segundo é nulo só quando $e_\theta = 0$ módulo $2\pi$ e **não** sofre com o salto de $\pm\pi$ da orientação (por isso o uso de $\cos$ em vez de $e_\theta^2$ com `wrapToPi`).

## 2.3 Derivadas de Lie

Para referência variante no tempo, $\dot V = L_f V + L_g V\, u + V_t$, com

$$
L_g V = \begin{bmatrix} \cos\theta\,(x - x_d) + \sin\theta\,(y - y_d) & \;\; c_\theta \sin(\theta - \theta_d) \end{bmatrix}
= \begin{bmatrix} -e_x & -c_\theta \sin e_\theta \end{bmatrix}
$$

$$
V_t = (x_d - x)\,\dot x_d + (y_d - y)\,\dot y_d - c_\theta\,\dot\theta_d \sin(\theta - \theta_d), \qquad L_f V = 0
$$

Essas expressões foram derivadas simbolicamente em [`matlab/common/LIE_DIF_CBF.m`](../matlab/common/LIE_DIF_CBF.m) e conferidas contra o código do bloco.

## 2.4 Restrição com folga

$$
L_g V\, u - \delta \;\le\; -c\,V - V_t
$$

A folga $\delta$ é penalizada no custo por $psc\,\delta^2$. Isso torna a CLF uma restrição **mole**: quando ela conflita com a segurança, vence a CBF (restrição dura) e $\delta$ absorve o conflito.

## 2.4.1 O que a CLF não faz sozinha

Em $L_g V$ **não aparece $e_y$**: a CLF só enxerga o erro à frente ($e_x$) e o de orientação ($e_\theta$). O erro lateral é reduzido de forma indireta, girando (restrição não holonômica). Dois efeitos observados no estudo:

1. Com o QP minimizando apenas $\|u\|^2$, o robô se estabilizava numa órbita de raio ≈ 1,65 m (erro lateral permanente de ≈ 0,35 m), mesmo longe dos obstáculos: a CLF mole aceitava o erro para gastar menos controle. A correção foi um **controle nominal** dentro do QP ([05-controle-nominal.md](05-controle-nominal.md)).
2. Para pose fixa, o robô pode parar num ponto em que $e_x = 0$ (alvo ao lado do robô), onde $L_g V = 0$ e a CLF não empurra mais ([05-controle-nominal.md](05-controle-nominal.md), seção 5.3).

## 2.5 Parâmetros

| Parâmetro | Papel | Valores usados |
|---|---|---|
| $c$ (`c`) | taxa de decaimento exigida de $V$ | 0,3 |
| $c_\theta$ (`ctheta`) | peso do erro de ângulo | 1 |
| $psc$ | peso da folga $\delta$ | 1000 (círculo), 100 (reta e ponto) |

## 2.6 Onde isso está no código

| MATLAB (bloco) | Python |
|---|---|
| `V`, `LgV`, `Vt`, `LfV = 0` | `controller.clf_terms` |
| linha da CLF em `Aqp`/`bqp`: `[LgV, -1]`, `-LfV - c*V - Vt` | `controller.solve_qp` |
