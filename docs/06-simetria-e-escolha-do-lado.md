# 6. Simetria, impasse e escolha do lado do desvio

## 6.1 O problema

Quando o obstáculo está exatamente à frente do robô (sobre o caminho), a situação é simétrica:

- $L_g h \approx [\,2d,\; 0\,]$: a CBF só limita $v$; nada empurra o robô para um lado.
- A CLF não distingue esquerda de direita.

O robô converge para a borda da região de segurança e **para** ali: um equilíbrio indesejado. É o mesmo tipo de fenômeno dos mínimos locais dos campos potenciais (Khatib, 1986; Koren e Borenstein, 1991) e dos equilíbrios assintoticamente estáveis introduzidos por QPs com CBF (Reis et al., 2021).

## 6.2 Estratégias testadas

| # | Estratégia | Resultado |
|---|---|---|
| 1 | **Viés de lado fixo** em $w_{nom}$, ativo perto de obstáculos à frente | Quebra a simetria. Mas o lado é sempre o mesmo, e o robô contorna por dentro obstáculos que poderiam ser contornados por fora. Uma versão com `persistent` (memória do lado) não pode ser usada no bloco contínuo do Simulink |
| 2 | **Viés direcional** (reta): só age com o obstáculo quase à frente, e empurra para o lado oposto a ele | Removeu o desvio desnecessário quando o obstáculo estava fora do caminho |
| 3 | **Viés por posição radial** (círculo): obstáculo dentro do círculo, vira à direita; fora, à esquerda | Funcionou com $r_s = 0{,}2$ m (erro até o ponto móvel de 0,323 para 0,129 m). **Não funcionou com $r_s = 0{,}3$ m** |
| 4 | **Referência interna deformada** ([07](07-referencia-deformada.md)) | Decisão determinística; reduziu o afastamento do círculo a menos da metade |

## 6.3 Por que o viés não forçou o lado com $r_s = 0{,}3$ m

Com $r_s = 0{,}3$ m, a trajetória do círculo invade a região de segurança de alguns obstáculos. Nos testes (Python), o robô passou por dentro de todos os obstáculos mesmo com:

- o viés mais forte ($w_b$ até 2) e mais cedo ($h_{act}$ até 3);
- a CLF mais fraca ($psc$ de 1000 até 1).

Conclusão do estudo: **quem decide o lado é a CBF junto com o estado do robô naquele instante**; o termo extra no controle nominal não controla essa decisão. Não foi isolada a causa exata; a suspeita é que o desvio do obstáculo anterior deixa o robô do lado de dentro quando o próximo chega. Esses experimentos fazem parte do histórico ([12](12-historico-de-melhorias.md)) e **não estão no pacote Python**, que implementa o viés fixo e o direcional.

## 6.4 A solução adotada

Em vez de empurrar o robô, **mover o alvo**: a referência que o controle persegue já passa do lado escolhido, e a CBF fica só como proteção. A regra é a do **menor desvio**: contornar pelo lado em que a referência precisa se deslocar menos. Veja [07-referencia-deformada.md](07-referencia-deformada.md).
