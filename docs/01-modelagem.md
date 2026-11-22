# 1. Modelagem do robô móvel diferencial

## 1.1 Modelo cinemático (monociclo)

O robô diferencial é modelado pela sua pose $q = (x, y, \theta)$ e por duas entradas: a velocidade linear $v$ e a velocidade angular $w$.

$$
\dot x = v\cos\theta, \qquad \dot y = v\sin\theta, \qquad \dot\theta = w
$$

Na forma afim no controle, $\dot q = f(q) + g(q)\,u$, com $u = (v, w)^\top$:

$$
f(q) = \begin{bmatrix}0\\0\\0\end{bmatrix}, \qquad
g(q) = \begin{bmatrix}\cos\theta & 0\\ \sin\theta & 0\\ 0 & 1\end{bmatrix}
$$

Como $f = 0$, todos os termos $L_f(\cdot)$ do restante do estudo são nulos. É o que simplifica as restrições do QP (`LfV = 0`, `Lfh = 0` nos blocos MATLAB).

**Restrição não holonômica.** O robô não anda de lado: $\dot x \sin\theta - \dot y \cos\theta = 0$. Duas consequências aparecem ao longo do estudo:

- o erro lateral ($e_y$) só é corrigido girando e avançando, nunca diretamente;
- a barreira de segurança precisa de uma entrada que "enxergue" o obstáculo (veja a seção 1.2).

## 1.2 Ponto de controle

A segurança é medida em um ponto $p$ situado a uma distância $l$ **à frente do eixo**:

$$
p = \begin{bmatrix} x + l\cos\theta \\ y + l\sin\theta \end{bmatrix}, \qquad
\dot p = \begin{bmatrix}\cos\theta & -l\sin\theta \\ \sin\theta & l\cos\theta\end{bmatrix} u
$$

A matriz é inversível para $l \neq 0$. Com $l = 0$, a distância ao obstáculo só depende de $x, y$, e a derivada de $h$ em relação a $w$ é **zero**: a barreira limitaria apenas $v$, e o robô não poderia contornar o obstáculo girando. Com $l > 0$, $w$ passa a ter autoridade sobre a barreira (seção 3 de [03-cbf.md](03-cbf.md)).

> **Consequência prática.** A CBF protege o **ponto** $p$, não o corpo inteiro. Se o robô real tem dispositivos acoplados além de $p$, o raio de segurança $r_s$ precisa cobrir o ponto mais distante do corpo a partir de $p$ (ver [13-guia-de-parametros.md](13-guia-de-parametros.md)).

## 1.3 Erros de pose no referencial do robô

Dada uma referência $(x_d, y_d, \theta_d)$:

$$
e_x = \cos\theta\,(x_d - x) + \sin\theta\,(y_d - y), \qquad
e_y = -\sin\theta\,(x_d - x) + \cos\theta\,(y_d - y), \qquad
e_\theta = \theta_d - \theta
$$

$e_x$ é o erro à frente do robô, $e_y$ é o erro lateral e $e_\theta$ é o erro de orientação. São usados na CLF ([02-clf.md](02-clf.md)) e nos controles nominais ([05-controle-nominal.md](05-controle-nominal.md)).

## 1.4 Discretização

Nas simulações, a integração é de Euler com passo $T_s = 0{,}01$ s. No Simulink, o bloco *MATLAB Function* roda com o mesmo período. O período importa: a garantia de segurança da CBF vale em tempo contínuo e só se mantém aproximadamente em tempo discreto, quando $T_s$ é pequeno (veja [13-guia-de-parametros.md](13-guia-de-parametros.md)).

## 1.5 Onde isso está no código

| Conceito | MATLAB | Python |
|---|---|---|
| $f$, $g$ | `f_model`, `g_model` no bloco | implícito em `controller.py` ($f=0$) |
| Ponto de controle | `px`, `py` no laço de obstáculos | `controller.cbf_terms` |
| Erros $e_x, e_y, e_\theta$ | `ex`, `ey`, `etheta` | `controller.pose_errors` |
| Integração | integradores do Simulink | `scenarios._euler` |
