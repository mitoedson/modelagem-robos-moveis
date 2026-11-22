# 14. Limitações e próximos passos

## 14.1 Limitações

- **Modelo cinemático.** Sem dinâmica do robô, atraso, ruído de localização nem limite de aceleração.
- **A CBF protege o ponto de controle**, não o corpo ([03](03-cbf.md)). Os valores de `rs` e `l` são de teste.
- **Obstáculos estáticos e conhecidos**, definidos no `INIT`. O "sensor" do ponto de chegada é simulado (conhece as posições exatas, usa só as ao alcance de 3 m).
- **Referência interna deformada.** O robô segue um alvo que não aparece no gráfico. Se o requisito for que a referência permaneça o círculo ou a reta originais e só a CBF atue, a escolha do lado volta a depender do viés e do estado ([06](06-simetria-e-escolha-do-lado.md)).
- **Regra do círculo:** centro na origem, sentido anti-horário, obstáculos separados por mais de ≈ 2,8 m de arco (para as deformações não se somarem) e a guarda do [9.7(a)](09-codigo-matlab.md).
- **Sem planejamento global.** Paredes ou becos sem saída podem prender o robô.
- **Período do bloco ≤ 0,01 s.**
- **Verificação no Simulink qualitativa**; os números quantitativos vêm da réplica Python. Os experimentos 🧪 do [histórico](12-historico-de-melhorias.md) não estão no pacote.
- **Variante do ponto com parâmetros** (`matlab/ponto/variantes`) não foi validada no Simulink.

## 14.2 Próximos passos

1. Aplicar os patches de [9.7](09-codigo-matlab.md) (guarda do círculo; corte do ponto de chegada) e validar no Simulink.
2. Calcular `rs` e `l` a partir das dimensões reais do robô e repetir as simulações.
3. Incluir atraso, ruído de localização, dinâmica e limite de aceleração no modelo.
4. Estudar uma referência que **espere o robô** durante o desvio (escala de tempo), reduzindo o atraso depois de cada obstáculo.
5. Obstáculos detectados em tempo real (alcance e campo de visão limitados), em vez de conhecidos de antemão.
6. Planejamento global para evitar becos sem saída.
7. Validação no robô real, em baixa velocidade, aumentando-a aos poucos.
8. Portar os experimentos descartáveis (viés radial, varreduras de parâmetros) para `python/examples`.
