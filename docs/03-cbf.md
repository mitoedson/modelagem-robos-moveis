# 3. Funções barreira de controle (CBF) e segurança

## 3.1 Conjunto seguro e condição da CBF

Seja $h(q)$ uma função cujo conjunto de nível $\mathcal C = \{q : h(q) \ge 0\}$ é o conjunto seguro. Uma *função barreira de controle* (Ames et al., 2017, 2019) é uma $h$ para a qual existe $u$ satisfazendo

$$
L_f h + L_g h\, u \;\ge\; -\alpha(h)
$$

Com $\alpha(h) = \lambda h$ ($\lambda > 0$), qualquer controle que satisfaça a condição mantém $\mathcal C$ **invariante para frente**: se o robô começa seguro, permanece seguro.

## 3.2 A barreira de um obstáculo circular

Para o obstáculo $i$ com centro $o_i$ e raio de segurança $r_s$, avaliada no ponto de controle $p$ (seção 1.2):

$$
h_i = \|p - o_i\|^2 - r_s^2 = (x + l\cos\theta - x_{o,i})^2 + (y + l\sin\theta - y_{o,i})^2 - r_s^2
$$

Com $d_x = p_x - o_{x,i}$ e $d_y = p_y - o_{y,i}$, e como $f = 0$:

$$
L_g h_i = \begin{bmatrix} 2 d_x\cos\theta + 2 d_y\sin\theta & \;\; 2\,l\,(d_y\cos\theta - d_x\sin\theta)\end{bmatrix}
$$

O primeiro termo é a taxa de aproximação ao obstáculo quando o robô **avança**. O segundo é a taxa quando o robô **gira**, proporcional a $l$.

### Por que $l > 0$

Com $l = 0$, o segundo componente é nulo: a barreira não depende de $w$. Com o obstáculo exatamente à frente, $L_g h_i \approx [\,2 d,\; 0\,]$ e a única ação da CBF é frear. Isso é parte da causa do impasse da [seção 6](06-simetria-e-escolha-do-lado.md).

## 3.3 Restrição no QP

$$
-L_g h_i\, u \;\le\; L_f h_i + \lambda h_i \;=\; \lambda h_i
$$

No MATLAB, uma linha por obstáculo: `Aqp(1+i,:) = [-Lghm(i,:), 0]` e `bqp(1+i) = Lfh + lambda*hv(i)`. Vários obstáculos formam a **interseção** dos conjuntos seguros; adicionar um obstáculo é só adicionar uma linha.

## 3.4 O que $\lambda$ faz

$\lambda$ define quão perto da borda o robô chega antes de a CBF frear. Para $\lambda$ grande, a restrição só atua perto da borda ($h$ pequeno) e o robô desvia tarde. Para $\lambda$ pequeno, ela atua cedo e o desvio começa longe. O estudo usou $\lambda = 10$. Nos testes de Python, variar $\lambda$ de 5 a 10 teve efeito pequeno na retomada depois do obstáculo.

## 3.5 O que a garantia cobre, e o que não cobre

| Cuidado | Explicação |
|---|---|
| **Só o ponto $p$** | A CBF protege $p$, não o corpo. Com o obstáculo atrás do eixo, o centro do robô pode chegar a $r_s - l$ do obstáculo. No gráfico, o robô pode parecer "entrar" na região pontilhada. |
| **Tempo discreto** | A garantia é contínua. Com passo grande (nos testes, $T_s = 0{,}5$ s), a barreira foi violada entre amostras ($h$ mínimo ≈ −0,08). Use $T_s \le 0{,}01$ s. |
| **Passagem estreita** | Dois obstáculos com menos de $2 r_s$ entre eles tornam a restrição incompatível com a passagem. Não é erro do código. |
| **Viabilidade** | Junto com os limites $|v|, |w| \le 2$, as CBFs podem ficar inviáveis em situações extremas. |
| **Obstáculos estáticos e conhecidos** | Todos os testes assumem isso. |

## 3.6 Onde isso está no código

| MATLAB (bloco) | Python |
|---|---|
| laço `for i = 1:nobs`: `hv(i)`, `Lghm(i,:)` | `controller.cbf_terms` |
| linhas das CBFs em `Aqp`/`bqp` | `controller.solve_qp` |
| `h = min(hv)` (só para o gráfico) | `h` mínimo devolvido por `solve_qp`; `metrics.min_h` |
