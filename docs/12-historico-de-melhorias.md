# 12. Histórico de problemas, modificações e melhorias

Cronologia do estudo, com a causa de cada problema, o que foi feito e o efeito. Itens marcados com 🧪 foram experimentos de *scripts* descartáveis (resultados descritos aqui, mas fora do pacote Python).

## 12.1 Ponto de partida

Controlador CLF+CBF resolvido por QP minimizando $\|u\|^2$, com **um** obstáculo no bloco. Problemas: o robô parava diante do obstáculo alinhado; afastava-se da trajetória mesmo longe dos obstáculos; o segundo obstáculo do `INIT` existia no gráfico, mas não no controlador.

## 12.2 Linha do tempo

| # | Problema observado | Causa | Modificação | Efeito |
|---|---|---|---|---|
| 1 | Robô para diante de obstáculo alinhado | Simetria ([06](06-simetria-e-escolha-do-lado.md)) | Viés de lado em $w$, ativo perto do obstáculo. Versão com `persistent` falhou no bloco contínuo; ficou lado fixo | O robô decide e contorna |
| 2 | Erro permanente ≈ 0,35 m (órbita de raio 1,65 m) | QP minimizava $\|u\|^2$ | QP minimiza $\|u - u_{nom}\|^2$ com Kanayama ([05](05-controle-nominal.md)) | Retorno à referência |
| 3 | Segundo obstáculo ignorado | Bloco com um obstáculo | CBF por obstáculo; `xo1`, `yo1` e depois vetores `xo`, `yo` ([09](09-codigo-matlab.md)) | N obstáculos sem editar o bloco |
| 4 | Obstáculo "duplicado" e viés dobrado; modelo errado sendo simulado | Valores escritos no bloco diferentes do `INIT`; nome do modelo ≠ chamada de `sim` | Obstáculos só no `INIT`; mesmo nome em `sim()` | Gráfico = o que o controle vê |
| 5 | Oscilação de $w$ (±2 rad/s) | $psc = 10000$, escolhido numa busca que só olhava o erro de posição | $psc = 1000$ ([04](04-qp-e-solver.md)); 200 iterações; $T_s = 0{,}01$ s | Saltos de $w$ de 520 para 12 |
| 6 | Reta: desvio indevido no 2º obstáculo | Viés fixo à esquerda mesmo com obstáculo fora do caminho | Viés só com obstáculo à frente, para o lado oposto (`directional_bias`) | Sem desvio desnecessário |
| 7 | Reta: retomada lenta | Correção lateral ∝ $v_d$ (0,14 m/s na reta) | $k_y$ de 4 para 10 a 15. 🧪 Desvio lateral em $x=3$ m: 0,23 → 0,03 m | Retoma antes do próximo obstáculo |
| 8 | Pose: parava em (4,6) em vez de (5,5) | Kanayama com $v_d = 0$ ([05](05-controle-nominal.md)) | Controlador polar | Alcança a pose |
| 9 | Lado do desvio imprevisível no círculo | CBF + estado decidem o lado | 🧪 Viés radial: funcionou com $r_s = 0{,}2$ m, **não** com 0,3 m (mesmo com $w_b \le 2$, $h_{act} \le 3$, $psc \ge 1$) | Motivou a referência deformada |
| 10 | Mesmo | | Referência interna deformada, regra do menor desvio ([07](07-referencia-deformada.md)) | Distância ao círculo: 0,196/0,557 → 0,091/0,231 m ($r_s = 0{,}3$) |
| 11 | Gráfico mostrava a referência deformada | | Saída do bloco passa a ser o **círculo original** | Gráfico mostra o desvio do robô |
| 12 | Obstáculo da direita contornado por fora com desvio maior | Tolerância $t_{olo} = 0{,}1$ preferia "fora" | $t_{olo} = 0$ (menor desvio) | 0,134/0,368 → 0,126/0,333 m |
| 13 | Reta: lado dependente de transitório | Viés dependia de pequena posição lateral | Referência deformada na reta, $p_w = 0{,}8$ m | Lado determinístico; desvio rms igual ou melhor |
| 14 | Ponto de chegada sem caminho | | Caminho virtual fixo S→G com alvo à frente. **Travou** com $L_d = 1$ m e deformação estreita | $L_d = 0{,}6$ m, $p_w = 1{,}2$ m |
| 15 | Ponto dependia da posição de partida fixa | Caminho a partir de $S$ fixo | Alvo reativo, recalculado da pose atual ("sensor") | Qualquer pose inicial, inclusive de costas |
| 16 | Orientação final errada (≈ 0,3 rad) e oscilação perto do alvo | Lei polar com $\rho \to 0$ | Giro no lugar quando $\|G - p\| < L_{rot}$ | 0,00 rad |
| 17 | Parava a 3–4 cm do ponto final | $L_{rot} = 0{,}08$ m | $L_{rot} = 0{,}005$ m | 0,5 cm. 🧪 0,08 → 0,02 → 0,005: 2,8–3,8 → 1,6 → 0,5 cm |
| 18 | Robô ia para (5,5) com o `INIT` em (3,4) | $G$ escrito no bloco e no `INIT` | Variante com `xg, yg, thg` como parâmetros (não validada) | Ponto em um lugar só ([09](09-codigo-matlab.md)) |
| 19 | Obstáculo atrás do alvo causava oscilação | Deslocava o alvo | Corte "só obstáculos antes de $G$" | 2187 → 0 saltos de $w$ |

## 12.3 Erros e correções ao longo do estudo

Registro honesto das hipóteses que se mostraram incorretas:

- A busca aleatória de parâmetros sugeriu $psc = 10000$ por reduzir o erro, sem medir a suavidade do controle. O chattering mostrou o erro (item 5).
- Na pose, supôs-se que o percurso curvo vinha do controle de pose; o teste **sem obstáculos** mostrou que vinha dos obstáculos. A causa do robô parado era outra (Kanayama com $v_d = 0$).
- Supôs-se que um viés mais forte forçaria o lado do desvio; não forçou com $r_s = 0{,}3$ m (item 9).
- Criar parâmetros novos editando o XML do `.slx` não foi validado no Simulink; o caminho que comprovadamente funcionou para vários obstáculos foi usar vetores nos parâmetros já existentes (`xo`, `yo`).

## 12.4 Lições

1. **Medir mais de uma coisa**: erro de rastreamento *e* suavidade do controle.
2. **Isolar variáveis**: rodar o cenário sem obstáculos separa o efeito do controle nominal do efeito da barreira.
3. **Um lugar para cada dado**: obstáculos e ponto de chegada definidos em um só arquivo evitam gráficos que não refletem a simulação.
4. **A CBF garante segurança, não comportamento**: a escolha do lado e a retomada dependem do controle nominal e da referência.
5. **Verificar a versão carregada**: `bdclose all`, nome exato do modelo e `which -all`.
