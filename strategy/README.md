# Estratégia

O módulo recebe `FieldPoses` via UDP multicast (`224.1.1.1:5007`) e envia
`TwistCmd` para um robô via UDP unicast.

O controlador atual tem três comportamentos:

- gira na direção da bola antes de acelerar;
- aproxima-se com velocidade proporcional à distância e reduz a velocidade
  perto do alvo;
- para dentro da distância de chegada e limita aceleração para evitar trancos.

Se a telemetria ficar 250 ms sem atualizar, a estratégia envia um comando de
parada e zera o estado do controlador. O comando volta a ser liberado quando
um frame válido chega.

## Configuração por ambiente

Os padrões são robô `0`, host `robot.local` e porta `3333`. Eles podem ser
alterados sem recompilar:

```bash
FUT_ROBOT_ID=1 FUT_ROBOT_HOST=robot-1.local FUT_ROBOT_PORT=3333 ./build/strategy
```

O ID aceito fica entre `0` e `3`.
