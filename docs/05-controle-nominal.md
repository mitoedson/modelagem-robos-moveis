# 5. Controle nominal

O QP minimiza $\|u - u_{nom}\|^2$ ([04-qp-e-solver.md](04-qp-e-solver.md)). A escolha de $u_{nom}$ define o comportamento longe dos obstáculos: se o nominal é bom, a segurança só corrige o necessário.

## 5.1 Seguimento de trajetória: Kanayama

Para trajetórias (círculo e reta), com velocidade de referência $v_d = \sqrt{\dot x_d^2 + \dot y_d^2}$ (Kanayama et al., 1990):

$$
v_{nom} = v_d\cos e_\theta + k_x\, e_x, \qquad
w_{nom} = \dot\theta_d + v_d\,\big(k_y\, e_y + k_\theta \sin e_\theta\big)
$$

| Ganho | Corrige | Efeito de aumentar |
|---|---|---|
| $k_x$ | erro à frente | alcança a referência mais rápido; $v$ mais agressivo |
| $k_y$ | erro lateral | volta à trajetória mais rápido; risco de oscilar |
| $k_\theta$ | erro de orientação | alinha mais depressa |

**O termo lateral é proporcional a $v_d$.** Na reta do estudo, $v_d = 0{,}14$ m/s; no círculo, 0,5 m/s. Com o mesmo $k_y$, a correção lateral na reta é ≈ 3,5 vezes mais fraca. Por isso os ganhos diferem: círculo $(k_x, k_y, k_\theta) = (4, 4, 6)$ e reta $(2, 10, 3)$. Em teste de reta, subir $k_y$ de 4 para 10 reduziu o desvio lateral em $x = 3$ m de 0,23 m para 0,03 m (experimento do histórico, [12](12-historico-de-melhorias.md)).

## 5.2 Regulação de pose: controlador polar

Para ir a um ponto com orientação final (Aicardi et al., 1995; Siegwart et al., 2011), com $\rho$ a distância ao alvo, $\alpha$ o ângulo do alvo em relação à frente do robô e $\beta$ o erro de orientação restante:

$$
v_{nom} = k_\rho\,\rho\,\max(\cos\alpha,\,0), \qquad w_{nom} = k_\alpha\,\alpha + k_\beta\,\beta
$$

com $k_\rho > 0$, $k_\beta < 0$, $k_\alpha > k_\rho$ (valores do estudo: $0{,}8$; $2$; $-0{,}5$).

Duas adaptações do estudo:

- **Sem ré**: $\max(\cos\alpha, 0)$. Se o alvo está atrás, o robô gira antes de avançar.
- **Giro no lugar**: quando a distância ao ponto final cai abaixo de $L_{rot}$ (5 mm), faz $v = 0$ e $w = k_{rot}(\theta_G - \theta)$. A lei polar não corrige bem a orientação quando $\rho \to 0$ ($\alpha$ fica indefinido).

Efeito de $L_{rot}$ no erro final de posição (experimento de Python, 8 casos):

| $L_{rot}$ | Erro final de posição | Erro de orientação |
|---|---|---|
| 0,08 m | 2,8 a 3,8 cm | 0,00 rad |
| 0,02 m | 1,6 cm | 0,00 rad |
| 0,005 m | **0,5 cm** | 0,00 rad |

## 5.3 Por que o Kanayama não serve para pose fixa

Com referência fixa, $v_d = 0$: $v_{nom} = k_x e_x$ e $w_{nom} = 0$. Os termos em $e_y$ e $e_\theta$ somem. O robô converge para o conjunto de pontos com $e_x = 0$, ou seja, o alvo exatamente ao lado, e para ali.

Caso real do estudo: alvo $(5, 5, \pi/4)$; o robô parou em $(4, 6)$, onde $e_x = 0$ (todos os pontos da reta $x + y = 10$ têm essa propriedade com $\theta = \pi/4$). Nesse ponto, $L_g V = 0$ e a CLF também não empurra. A troca para o controlador polar resolveu.

## 5.4 Onde isso está no código

| | MATLAB (bloco) | Python |
|---|---|---|
| Kanayama | `kx, ky, kth`; `v_nom`, `w_nom` (circular e reta) | `nominal.kanayama` |
| Polar com giro no lugar | `kp, ka, kb, krot`; `Lrot` (ponto) | `nominal.polar` |
