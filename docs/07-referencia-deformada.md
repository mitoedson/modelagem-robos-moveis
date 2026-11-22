# 7. Referência interna deformada

O controle persegue uma referência **deslocada lateralmente em torno de cada obstáculo**. O deslocamento deixa a folga $r_s + m$ entre a referência e o obstáculo, pelo lado de **menor desvio**. A referência **exibida** nos gráficos continua sendo a original (círculo ou reta); o desvio mostrado é o do robô.

## 7.1 O "morro" de cosseno elevado

$$
b(u) = \tfrac12\big(1 + \cos(\pi u)\big), \qquad |u| < 1
$$

É suave (derivada contínua), vale 1 no centro e 0 nas bordas. É centrado no obstáculo, com meia-largura $p_w$.

## 7.2 Círculo (raio $R$, centro na origem, sentido anti-horário)

Referência em coordenadas polares, $r(\varphi) = R + \sum_i \delta_i\, b\!\left(\dfrac{\varphi - \varphi_i}{p_w}\right)$, com $\varphi = \omega t$ e $\varphi_i$ o ângulo do obstáculo $i$.

Com $r_o$ a distância do obstáculo ao centro e $m$ a folga:

| Condição | Lado | Deslocamento do raio |
|---|---|---|
| $r_o \le R + t_{olo}$ | por **fora** | $\delta = r_o + r_s + m - R$ |
| $r_o > R + t_{olo}$ | por **dentro** | $\delta = r_o - r_s - m - R$ |

Com $t_{olo} = 0$, é exatamente o lado de menor $|\delta|$. Obstáculo sobre a circunferência ($r_o = R$) cai em "fora" (a direita do sentido de movimento).

**Tolerância $t_{olo}$.** O estudo começou com $t_{olo} = 0{,}1$ m para preferir "por fora" quando o obstáculo está sobre a trajetória. Com o obstáculo em $(1{,}75;\,-1)$ ($r_o = 2{,}016$), isso forçava o contorno por fora ($\delta = 0{,}366$ m) quando por dentro bastava $0{,}334$ m. Com $t_{olo} = 0$, o lado é sempre o de menor desvio. Compare em [`figures/simulink/`](../figures/simulink) e [`figures/python/circular_tolerancia_regra_lado.png`](../figures/python/circular_tolerancia_regra_lado.png).

Para as derivadas (necessárias para $\dot x_d$, $\dot y_d$, $\dot\theta_d$ na CLF), em curva polar $r(\varphi)$ a orientação da tangente é $\psi = \varphi + \operatorname{atan2}(r,\, r')$, e

$$
\dot\theta_d = \omega\left(1 + \frac{r'^2 - r\,r''}{r^2 + r'^2}\right)
$$

com $r' = \sum_i \delta_i\, b'$ e $r'' = \sum_i \delta_i\, b''$, calculados em `d1` e `d2` no código.

## 7.3 Reta (de $S$ a $G$, velocidade constante)

No referencial da reta, com abscissa $s$ e normal $n$ (à esquerda): $p(s) = S + s\,\hat u + \ell(s)\,n$. Para cada obstáculo, com $s_o$ e $d_{lat}$ sua abscissa e distância lateral (com sinal):

- se $|d_{lat}| \ge r_s + m$: o obstáculo não bloqueia; sem deslocamento;
- senão: $\ell^\ast = d_{lat} - \operatorname{sgn}(d_{lat})\,(r_s + m)$, que leva a referência ao lado **oposto** ao obstáculo (o de menor desvio); se $d_{lat} \approx 0$, passa pela direita ($\ell^\ast = -(r_s + m)$).

$\ell(s) = \sum_i \ell^\ast_i\, b((s - s_{o,i})/p_w)$, com $p_w = 0{,}8$ m. A orientação é $\theta_d = \operatorname{atan2}(\dot y_d, \dot x_d)$ e $\dot\theta_d = v_d\,\ell''/(1 + \ell'^2)$.

$p_w = 1{,}4$ m (mais largo) piorou o desvio rms (0,174 contra 0,132 m no cenário B do relatório); ficou-se em 0,8 m.

## 7.4 Ponto de chegada: alvo reativo, sem trajetória

Não há caminho fixo. A cada instante, usando só a pose do robô $p$, o ponto de chegada $G$ e os obstáculos ao alcance do "sensor" ($R_{sen} = 3$ m):

1. Direção até $G$: $\hat u = (G - p)/\|G - p\|$. O alvo está a $L_d = 0{,}6$ m à frente nessa direção.
2. Cada obstáculo com $-0{,}3 < s_o < \min(R_{sen},\, \|G - p\| + 0{,}3)$ e $|d_{lat}| < r_s + m$ desloca o alvo lateralmente (mesma regra de lado). O corte em $\|G - p\| + 0{,}3$ ignora obstáculos **atrás** do ponto de chegada.
3. A orientação final entra nos últimos $L_b = 1{,}5$ m (fator $\lambda_b = 1 - \|G - p\|/L_b$, limitado a $[0,1]$).
4. Perto de $G$, o robô gira no lugar ([05](05-controle-nominal.md)).

O alvo é recalculado a cada instante a partir da pose atual, sem memória de estado. Raciocínio (não demonstrado formalmente): ao se deslocar para o lado oposto ao obstáculo, o robô aumenta $|d_{lat}|$ em relação à nova direção até $G$, e o obstáculo continua do mesmo lado, então a escolha tende a se manter. Nos testes de Python, não houve inversão de lado.

**Dois cuidados descobertos no estudo:**

- $L_d$ longo e deformação estreita fizeram o robô **travar** diante de um obstáculo sobre a reta (o alvo ficava além da deformação). Com $L_d = 0{,}6$ m e $p_w = 1{,}2$ m, não trava.
- Um obstáculo logo atrás do alvo (sem o corte do passo 2) deslocava o alvo e causou 2187 saltos bruscos de $w$ e 0,027 rad de erro de orientação (Python).

## 7.5 Guarda para obstáculos longe do círculo (importante)

No bloco do **círculo**, o deslocamento $\delta$ é aplicado a **todos** os obstáculos, sem verificar se eles realmente bloqueiam o círculo. Um obstáculo longe da circunferência (por exemplo, em $(0, 1)$, a 1 m do centro) gera $\delta = r_o + r_s + m - R = -0{,}65$ m, e a referência é puxada **em direção** ao obstáculo, sem necessidade.

Experimento (Python, `python/examples/make_figures.py`):

| Obstáculo único em $(0, 1)$ | Distância ao círculo (rms / máx.) | $h$ mínimo |
|---|---|---|
| sem a guarda (como no bloco atual) | 0,185 / 0,650 m | 0,003 |
| com a guarda $|r_o - R| < r_s + m$ | 0,002 / 0,005 m | 0,907 |

Nos cenários do estudo, todos os obstáculos ficam a menos de $r_s + m = 0{,}35$ m da circunferência ($r_o \in [1{,}8;\;2{,}2]$), então a guarda não altera os resultados e o problema não apareceu. A reta e o ponto de chegada **já** têm essa guarda. O patch para o bloco MATLAB está em [09-codigo-matlab.md](09-codigo-matlab.md), seção 9.7, e o pacote Python tem `CircleRef(guard=True)` por padrão.
