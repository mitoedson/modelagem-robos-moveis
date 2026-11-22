# 9. Estudo do código MATLAB

## 9.1 Visão geral dos arquivos

```
matlab/
├── setup.m                      # adiciona matlab/common ao caminho do MATLAB
├── common/
│   ├── QPhild.m                 # solver de Hildreth (200 iterações)
│   └── LIE_DIF_CBF.m            # derivação simbólica (Symbolic Math Toolbox)
├── circular/
│   ├── INIT_DIF_CBF_circular.m  # parâmetros, obstáculos, chamada de sim() e gráficos
│   ├── SIMU_DIF_CBF_v2_refcirc.slx
│   └── fcn_circular_bloco.m     # cópia LEGÍVEL do código do bloco (só leitura)
├── reta/    (idem: INIT_DIF_CBF_reta_refdef.m, SIMU_DIF_CBF_reta_refdef.slx, fcn_reta_bloco.m)
└── ponto/   (idem: INIT_DIF_CBF_ponto_sensor.m, SIMU_DIF_CBF_ponto_sensor.slx, fcn_ponto_bloco.m)
```

## 9.2 Fluxo de execução

1. `INIT_*.m` limpa o workspace, define **pose inicial**, **obstáculos** e **parâmetros**.
2. `sim('<modelo>', tspan)` roda o Simulink. Os integradores usam `x0`, `y0`, `theta0` como condições iniciais.
3. O bloco *MATLAB Function* lê `u = [x; y; theta; t]` e os **parâmetros** do workspace, calcula o controle e devolve `out` (9 sinais).
4. Os sinais voltam ao workspace (`To Workspace`) e o próprio `INIT` desenha as figuras 1 (sinais) e 2 (trajetória).

Por isso o modelo **não roda sozinho pelo botão Play**: sem o `INIT`, faltam `x0`, `c`, `xo`, etc. (veja a seção 9.8).

## 9.3 Interface do bloco

| Elemento | Tipo no Edit Data | Observação |
|---|---|---|
| `u` | Input | vetor 4×1: `x, y, theta, t` |
| `out` | Output | vetor 9×1: `x', y', theta', v, w, h, xd0, yd0, thetad0` |
| `xo`, `yo` | **Parameter** | tamanho herdado: aceitam **vetores** (um elemento por obstáculo) |
| `l`, `rs`, `psc`, `c`, `lambda` | Parameter | escalares do workspace |

Assinatura (circular e reta): `function out = fcn(u, xo, yo, l, rs, psc, c, lambda)`. Escalar ou vetor, o bloco usa `obs = [xo(:), yo(:)]; nobs = size(obs,1);`. Para um obstáculo a mais, basta incluir um valor nos vetores do `INIT` (e manter `Ro` do mesmo tamanho).

## 9.4 Anatomia do código do bloco (circular)

Leia em paralelo com [`circular/fcn_circular_bloco.m`](../matlab/circular/fcn_circular_bloco.m).

| Seção | Variáveis principais | Equivale a |
|---|---|---|
| Entradas e modelo | `x, y, theta, t`, `f_model`, `g_model` | [01](01-modelagem.md) |
| Obstáculos | `obs`, `nobs` | lista de obstáculos |
| **Referência deformada** | `R, w_ref_const, phi, Rr, d1, d2, m, pw, tolo`; saída: `xd, yd, xd_dot, yd_dot, thetad, thetad_dot` | [07](07-referencia-deformada.md) |
| **CLF** | `ex, ey, etheta, V, LfV, LgV, Vt` | [02](02-clf.md) |
| **CBFs e viés** | laço `for i = 1:nobs`: `hv(i)`, `Lghm(i,:)`, `wbias`; `h = min(hv)` | [03](03-cbf.md) |
| **Controle nominal** | `kx, ky, kth`, `vd`, `v_nom`, `w_nom`, `u_nom` | [05](05-controle-nominal.md) |
| **QP** | `Hqp`, `Fqp`, `Aqp`, `bqp`; `ucont = QPhild(...)` | [04](04-qp-e-solver.md) |
| Saída | `xmponto = f_model + g_model*uc`; `out = [...]` | modelo $\dot q = f + g u$ |

Pontos de atenção do código:

- `coder.extrinsic('QPhild')` chama o `QPhild.m` fora da geração de código; a saída precisa ser pré-alocada (`ucont = [0;0;0]`).
- `wb = 0.0` nos três blocos: o viés de lado está **desligado** (a referência deformada escolhe o lado). O código do viés continua presente.
- `h = min(hv)` só alimenta o gráfico; a segurança vem das linhas de `Aqp`.
- O bloco devolve a referência **original** em `xd0, yd0, thetad0` (o controle usa a deformada internamente).

## 9.5 Diferenças entre os três blocos

| | Circular | Reta | Ponto de chegada |
|---|---|---|---|
| Referência | círculo deformado, função de `t` | reta deformada, função de `t` | alvo reativo, função de `(x, y, theta)` |
| Nominal | Kanayama | Kanayama | polar com giro no lugar |
| `xd_dot, yd_dot, thetad_dot` | calculados | calculados | **zero** ($V_t = 0$) |
| Constantes dentro do bloco | `R, w_ref_const`, `m, pw, tolo`, ganhos | `S, G, Tf`, `m, pw`, ganhos | `G, thG`, `m, pw, Ld, Lb, Rsen, Lrot`, ganhos |

No ponto de chegada, a "referência" da CLF é o próprio alvo móvel (com $V_t = 0$), e o controle nominal polar é o que efetivamente leva o robô ao ponto.

## 9.6 `QPhild.m` e `LIE_DIF_CBF.m`

- [`common/QPhild.m`](../matlab/common/QPhild.m): ver [04](04-qp-e-solver.md). A diferença para a versão inicial do estudo é só `for km=1:200` (era 38).
- [`common/LIE_DIF_CBF.m`](../matlab/common/LIE_DIF_CBF.m): usa a *Symbolic Math Toolbox* para derivar `LgV`, `Vt`, `Lgh` e confere que as expressões escritas à mão no bloco estão corretas. Considera **um** obstáculo (`xo`, `yo`) e guarda definições antigas comentadas; serve como **verificação**, não como parte da simulação.

## 9.7 Pontos de atenção e patches propostos

Três diferenças entre os modelos entregues e a versão descrita no relatório e no pacote Python. **Os arquivos `.slx` do repositório não foram alterados**; os patches abaixo são sugestões.

### (a) Circular: guarda para obstáculos longe do círculo

O laço aplica $\delta$ a todos os obstáculos ([07, seção 7.5](07-referencia-deformada.md)). Envolva o corpo do laço da referência com a guarda:

```matlab
for i = 1:nobs
    ro = sqrt(obs(i,1)^2 + obs(i,2)^2);
    ph = atan2(obs(i,2), obs(i,1));
    if abs(ro - R) < rs + m                 % NOVO: só deforma se o obstáculo bloqueia o círculo
        if ro <= R + tolo
            delta = ro + rs + m - R;
        else
            delta = ro - rs - m - R;
        end
        dphi = atan2(sin(phi - ph), cos(phi - ph));
        if abs(dphi) < pw
            Rr = Rr + delta*0.5*(1 + cos(pi*dphi/pw));
            d1 = d1 - delta*0.5*sin(pi*dphi/pw)*pi/pw;
            d2 = d2 - delta*0.5*cos(pi*dphi/pw)*(pi/pw)^2;
        end
    end                                      % NOVO
end
```

Não altera os cenários atuais (todos a menos de 0,35 m da circunferência).

### (b) Ponto de chegada: corte de obstáculos atrás do alvo

O modelo entregue tem `if so > -0.3 && so < Rsen && abs(dlat) < cm`. Troque por:

```matlab
if so > -0.3 && so < min(Rsen, Lg + 0.3) && abs(dlat) < cm   % só obstáculos ao alcance e antes de G
```

Só importa quando há obstáculo logo depois do alvo ([07, seção 7.4](07-referencia-deformada.md)). O pacote Python tem esse corte por padrão (`GoalConfig.cutoff = True`); com `cutoff=False`, reproduz o modelo entregue.

### (c) Ponto de chegada: `G` e `thG` escritos no bloco

No modelo entregue, o bloco tem `G = [5; 5]; thG = pi/4;` e o `INIT` repete `xg, yg, thg` só para o desenho. Se mudar um, mude o outro. Uma variante em que o bloco lê `xg, yg, thg` do workspace (como `xo`, `yo`) está em [`matlab/ponto/variantes/`](../matlab/ponto/variantes). **Foi gerada editando o XML do modelo e não foi validada no Simulink.** Veja o `README` da pasta.

## 9.8 Erros comuns encontrados durante o estudo

| Mensagem ou sintoma | Causa | Solução |
|---|---|---|
| `Unrecognized function or variable 'x0'` / `'c'` ao simular | Modelo rodado sem o `INIT` (workspace vazio) | Rodar o `INIT_*.m` no editor do MATLAB |
| Erro de `persistent` em bloco contínuo (*"writing to initialized persistent variables"*) | Memória de lado em bloco de tempo contínuo | Evitar `persistent`; ou bloco discreto |
| `Undefined function or variable 'xo1'` | Parâmetro criado como **Input** em vez de **Parameter**, ou não declarado | Declarar em *Edit Data* com **Scope = Parameter**, ou usar vetores `xo`, `yo` |
| *Edit Data* abre com `Dot indexing is not supported for variables of this type` | Variável do workspace conflita com a interface | `bdclose all; clear all; clc`, abrir o modelo **sem** rodar o `INIT` |
| `INIT` chama um modelo, mas o editado é outro | Nome do arquivo diferente do `sim('...')`; arquivo baixado como `... (1).slx` | Nome **exato**; `which <modelo> -all` deve listar uma só cópia |
| Resultado igual ao anterior, "sem efeito" | Versão antiga carregada na memória | `bdclose all` antes de simular |
| Texto do `INIT` colado dentro do bloco (`clear all`, `sim(...)` no bloco) | Colagem no editor errado | Restaurar o código do bloco |
| `Error using *` no `INIT`, na linha do plot dos obstáculos | `Ro` agora é vetor; sobrou o `plot` antigo (`Ro*cos(ang)`) | Usar o laço `for k = 1:length(xo)` do `INIT` atual |
| Oscilações de $w$ entre ±2 | $psc = 10000$ | $psc = 1000$ ou menos |
| Obstáculo ignorado (um dos dois) | Dois obstáculos com o mesmo ponto, ou valores no bloco diferentes do `INIT` | Obstáculos só no `INIT`, em vetores |
| Robô não atinge o ponto final | Controle nominal de trajetória usado para pose fixa | Controlador polar |

## 9.9 Como executar

1. Abra o MATLAB na pasta `matlab/` e rode `setup` (adiciona `common/` ao caminho; `QPhild` precisa estar no caminho).
2. Entre na pasta do cenário (`cd circular`, `cd reta` ou `cd ponto`).
3. `bdclose all` (evita versão antiga na memória).
4. Abra o `INIT_*.m` **no editor** e clique em **Run**.
5. Confira: o modelo deve estar com **Sample time = 0,01 s** (bloco *MATLAB Function*); o `h(t)` não deve ficar abaixo de zero.

Requisitos: MATLAB com Simulink; `wrapToPi` (Mapping Toolbox); a *Symbolic Math Toolbox* só para o `LIE_DIF_CBF.m`.
