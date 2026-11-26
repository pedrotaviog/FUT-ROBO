# Parâmetros do robô FUT-ROBO

Tudo o que foi medido e calibrado no robô, com os valores em uso no código, de
onde cada número veio e como medir de novo. Medições feitas em 05/10/2026.

> Os valores dos motores dependem da **bateria**: com a bateria fraca, o mesmo
> PWM chegou a dar metade da velocidade (ver "Bateria"). Toda calibração foi
> feita com a bateria carregada.

---

## 1. Hardware

| Item | Valor | Observação |
|---|---|---|
| Driver dos motores | Ponte H **L298N** | Os dois pinos em nível baixo (PWM 0) **freiam** o motor; não deixam a roda solta |
| Apoio | 2 rodas motoras + **2 rodízios giratórios** (rodas bobas) na frente | Os rodízios atrapalham giros em torno de uma roda só (pivô) |
| Bitola (distância entre os centros das rodas) | **0,19 m** | Medida com régua, aproximada |
| Centro da tag até a frente do robô | **~12,1 cm** | Difícil de medir; aproximado |
| Desvio da tag em relação à frente do robô | **~0°** (−0,7° a +1,6°) | Medido no teste de motores (linha reta) |
| Microcontrolador | ESP32 (ESP-IDF) | Nome na rede: `robot.local` |

### Pinos (firmware/main/main.c)

| Motor | Pinos | Canais LEDC | Observação |
|---|---|---|---|
| Direito (`MD`) | IN1 = GPIO 4, IN2 = GPIO 5 | 0 e 1 | Sentido invertido no software: `set_PWM(&MD, -pwm_r)` |
| Esquerdo (`ME`) | IN3 = GPIO 6, IN4 = GPIO 7 | 2 e 3 | |

- PWM: **1 kHz**, resolução de **12 bits** (o valor 0-255 é escalado para 0-4095).
- Convenção: PWM positivo = roda girando para a frente do robô (já com a inversão do motor direito).
- Sentido de cada roda conferido nos testes: **correto** nas duas.

---

## 2. Motores

### Limiares de PWM

| Medida | Valor | O que significa |
|---|---|---|
| **Parada** | **37 a 43** | Menor PWM que mantém a roda girando. Abaixo disso, a roda já girando para |
| **Partida** | **46 a 68** | PWM necessário para a roda sair do repouso (atrito estático) |

Medições individuais (PWM descontando o atraso da visão):

| Teste | Caso | Partida | Parada |
|---|---|---|---|
| v1 (pivô em uma roda) | esquerda frente / trás | 60 / 50 | 39 / 37 |
| | direita frente / trás | 58 / (inválido) | 37 / 43 |
| v2 (pivô em uma roda) | esquerda frente / trás | 68 / 54 | 41 / 43 |
| | direita frente / trás | 66 / 46 | 37 / 39 |
| **v3 final (giro no lugar)** | anti-horário / horário | **64 / 46** | **37 / 39** |

### Curva PWM → velocidade da roda (bateria carregada)

Medida no giro no lugar (as duas rodas no mesmo PWM, sentidos opostos),
subindo e descendo o PWM, nos dois sentidos de giro. Velocidade da roda em m/s:

| PWM | 40 | 50 | 60 | 80 | 100 | 130 | 160 | 200 | 255 |
|---|---|---|---|---|---|---|---|---|---|
| anti-horário, subindo | 0 | 0,018 | 0,071 | 0,118 | 0,150 | 0,197 | 0,238 | 0,284 | 0,351 |
| anti-horário, descendo | 0 | 0,043 | 0,074 | 0,126 | 0,160 | 0,205 | 0,240 | 0,291 | 0,351 |
| horário, subindo | 0 | 0 | 0,077 | 0,121 | 0,150 | 0,197 | 0,237 | 0,293 | 0,358 |
| horário, descendo | 0 | 0,053 | 0,083 | 0,130 | 0,159 | 0,204 | 0,246 | 0,295 | 0,358 |
| **média (usada no firmware)** | 0 | **0,048** | **0,076** | **0,124** | **0,155** | **0,201** | **0,240** | **0,291** | **0,354** |

- **A curva não é reta**: sobe rápido entre PWM 40 e 80 e fica menos inclinada depois.
  Por isso o firmware usa uma tabela com interpolação, não `PWM_MIN` + `MAX_SPEED`.
- **Velocidade máxima da roda: ~0,354 m/s** (PWM 255).
- **Menor velocidade contínua:** ~0,048 m/s na calibração. Na prática, saindo do
  repouso com PWM 50, a roda andou **0,02 a 0,03 m/s**.
- Em PWM 50, subindo a partir do repouso a roda quase não anda; descendo, ela já
  está girando e anda ~0,05 m/s. Isso é a histerese entre partida e parada.

### Comparação entre as rodas (linha reta, as duas no mesmo PWM)

| PWM | Sentido | v (m/s) | w (rad/s) | Roda esq. (m/s) | Roda dir. (m/s) | dir/esq | Desvio da tag |
|---|---|---|---|---|---|---|---|
| 80 | trás | −0,124 | −0,050 | −0,119 | −0,128 | 1,08 | −0,5° |
| 80 | frente | +0,117 | +0,016 | +0,116 | +0,119 | 1,03 | +1,0° |
| 100 | trás | −0,157 | +0,034 | −0,160 | −0,154 | 0,96 | +1,6° |
| 100 | frente | +0,156 | +0,062 | +0,150 | +0,162 | 1,08 | −0,7° |
| 120 | trás | −0,189 | 0,000 | −0,189 | −0,189 | 1,00 | +0,3° |
| 120 | frente | +0,189 | +0,062 | +0,183 | +0,194 | 1,06 | −0,2° |

- **As duas rodas são praticamente iguais:** razão direita/esquerda **1,03** (0,96 a 1,08).
- As velocidades da linha reta batem com as do giro no mesmo PWM, o que confirma
  a bitola de 0,19 m e a escala do campo.

### Bateria

Os mesmos PWMs (83 a 104), antes e depois de carregar a bateria (teste de giro, firmware antigo):

| Situação | Giro medido |
|---|---|
| Bateria mais baixa | 0,8 a 1,2 rad/s |
| Bateria carregada | 1,8 a 2,0 rad/s |

A velocidade chega a **dobrar** com a bateria. Se o robô ficar mais lento que o
normal, carregue a bateria antes de mexer em ganhos ou na calibração.

---

## 3. Firmware (firmware/main/main.c)

| Parâmetro | Valor | Função |
|---|---|---|
| `TRACK_WIDTH` | 0,19 m | Bitola, usada na cinemática (v, w) → velocidade de cada roda |
| `CAL_SPEED` / `CAL_PWM` | tabela da média acima (8 pontos, PWM 50 a 255) | Conversão velocidade da roda → PWM, com interpolação |
| `V_DEADZONE` | 0,01 m/s | Abaixo disso a roda fica parada (PWM 0) |
| Menor velocidade | 0,01 a 0,048 m/s → PWM 50 | Não existe velocidade contínua menor |
| `KICK_PWM` | 65 | Tranco de partida: PWM mínimo ao sair do repouso ou inverter o sentido |
| `KICK_MS` | 60 ms | Duração do tranco |
| `CMD_TIMEOUT_MS` | 300 ms | Failsafe: sem comando por esse tempo, os motores param |
| Log por pacote | nível DEBUG | Em INFO, duas linhas por comando a ~25 Hz pesavam na serial |
| Porta UDP | 3333 | Recebe `TwistCmd` (8 bytes) ou `MotorPwmCmd` (4 bytes) |

Comandos aceitos (`common/include/common/protocol.h`):
- `TwistCmd {float v; float w;}`: uso normal, em m/s e rad/s. Passa pela tabela e pelo tranco.
- `MotorPwmCmd {int16 left; int16 right;}`: PWM direto (−255 a 255), só para
  calibração. Não passa pela tabela nem pelo tranco.

---

## 4. Visão

| Parâmetro | Valor | Onde |
|---|---|---|
| Campo | **237 cm (x, horizontal da imagem) × 165 cm (y)** | `vision/config.yaml`, seção `field` |
| Coordenadas publicadas | cm reais, inteiros, origem no centro do campo | `Vec2` em `protocol.h` |
| Câmera | 1280 × 720, configurada para 30 fps | `vision/config.yaml` |
| Taxa real da visão | **~24,5 fps** | Medida nos testes |
| Cantos clicados (pixels) | (100, 11), (1112, 12), (1119, 707), (106, 715) | `vision/data/calib.yaml` |
| Ângulo do robô | direção do centro do retângulo ao centro do círculo da tag | `vision/src/main.cpp` |
| Posição do robô | centro do retângulo da tag | |
| Diâmetro da bola | **3,9 cm** | |

- Calibração do campo: os 4 cantos são clicados em sentido horário, começando
  pelo **superior esquerdo** da imagem. O lado entre o 1º e o 2º canto mede `sizeX_cm`.
  - **A câmera mexeu?** Rode a visão com `--calibrar` e clique os cantos de novo.
  - **O campo mudou de tamanho?** Só edite `field.sizeX_cm` / `field.sizeY_cm` no `config.yaml`.
- Com o robô girando, a visão quase não repete frames (0 a 5%). Com o robô
  parado, ela pode repetir o mesmo ângulo legitimamente.

### Problema histórico corrigido: escala do campo

Até 05/10/2026, o campo estava definido como 50 × 60 "cm" (x = 50, y = 60),
quando o real é 237 × 165. Cada eixo tinha uma escala diferente
(1 unidade = 4,74 cm em x e 2,75 cm em y), o que:
- **distorcia todos os ângulos** (até ~15°, conforme a orientação do robô);
- fazia o ganho do controle de ângulo variar até 3× com a orientação;
- deixava a posição com resolução de 3 a 5 cm em vez de 1 cm;
- tornava sem sentido todas as distâncias do `strategy` (parada, rampa etc.).

Backup da calibração antiga: `vision/data/calib.yaml.bak-escala-50x60`.

---

## 5. Atraso do sistema

| Medida | Valor |
|---|---|
| Atraso comando → movimento visto pela visão | **~0,35 s** (0,28 a 0,66 s) |

O atraso vem de câmera + visão + rede + motor. Tirar o log pesado da serial
**não** reduziu o atraso, então a causa provável está na câmera ou na rede
(ainda não investigado).

---

## 6. Desempenho medido com a calibração final (teste de giro)

| w pedido (rad/s) | PWM | Giro medido (rad/s) | medido/pedido | Girou após parar |
|---|---|---|---|---|
| +0,20 / −0,20 | 50 | +0,352 / −0,236 | 1,76 / 1,18 | +0,032 / −0,027 rad |
| +0,40 / −0,40 | 50 | +0,274 / −0,344 | 0,68 / 0,86 | +0,048 / −0,029 rad |
| +0,60 / −0,60 | 53 | +0,438 / −0,395 | 0,73 / 0,66 | +0,087 / −0,086 rad |
| +0,80 / −0,80 | 60 | +0,686 / −0,672 | 0,86 / 0,84 | +0,177 / −0,142 rad |
| +1,00 / −1,00 | 68 | +1,009 / −0,877 | 1,01 / 0,88 | +0,257 / −0,226 rad |
| +1,50 / −1,50 | 92 | +1,474 / −1,347 | 0,98 / 0,90 | +0,367 / −0,380 rad |

- De 0,8 rad/s para cima, o robô gira entre 84% e 101% do pedido.
  Com o firmware antigo, ia de 130% a 880%.
- Menor giro no lugar: **~0,25 a 0,35 rad/s**.
- Na faixa baixa (PWM 50 a 53), ele fica ~30% abaixo da tabela.

---

## 7. Strategy (strategy/include/strategy/ControlParams.hpp)

| Parâmetro | Valor | Função |
|---|---|---|
| `TRACK_WIDTH_M` | 0,19 m | Igual ao firmware |
| `MAX_WHEEL_SPEED_MS` | 0,354 m/s | Igual ao último ponto da tabela do firmware |
| `VISION_TO_M` | 0,01 | A visão publica cm |
| `CONTROL_DELAY_S` | 0,40 s | Atraso efetivo (0,35 s medido + ~0,05 s dos filtros), base da sintonia |
| `MIN_TURN_RATE` | 0,30 rad/s | Menor giro no lugar que o robô mantém (medido) |
| `KP_ANGULAR` | 0,7 (rad/s)/rad | = 0,28 / atraso. Integrador com atraso não oscila com Kp·atraso ≤ 1/e (~0,37); 0,28 dá margem (ver SIMC) |
| `KD_ANGULAR` | 0,15 | Ganho derivativo, sobre a velocidade de giro **medida** (não sobre o erro) |
| `MAX_ANGULAR_VEL` | 2,0 rad/s | |
| `MAX_LINEAR_VEL` | 0,3 m/s | |
| `ALIGN_IN` / `ALIGN_OUT` | 0,06 / 0,12 rad | Zona morta com histerese, só com o robô parado na bola. OUT = 2·(giro mínimo·atraso − IN): ele ainda gira ~0,12 rad depois de parar |
| `ANGLE_FILTER_ALPHA` | 0,5 | Filtro do ângulo do robô |
| `RATE_FILTER_ALPHA` | 0,3 | Filtro da velocidade de giro (termo D) |
| `POSITION_FILTER_ALPHA` | 0,4 | Filtro das posições da bola e do robô |
| `POSITION_JUMP_RESET_M` | 0,03 m | Salto maior que isso reinicia o filtro (bola chutada) |
| `ADVANCE_CONE` | 0,6 rad | Só avança com o erro de ângulo abaixo disso |
| `STOP_DISTANCE_M` | 0,20 m | Encostar seria ~0,14 m (12,1 cm tag → frente + 1,95 cm raio da bola); 0,20 ajustado em teste |
| `SLOW_DISTANCE_M` | 0,40 m | Começo da rampa de desaceleração |
| `MIN_APPROACH_VEL` | 0,08 m/s | Velocidade no fim da rampa |
| `MIN_TURN_CMD` | 0,15 rad/s | Compensação de zona morta: girando no lugar, o comando nunca fica abaixo disso (abaixo de ~0,105 rad/s o firmware zera as rodas) |
| Rede | multicast 224.1.1.1:5007 (visão), unicast `robot.local`:3333 | |

### Modo "ir para trás da bola, virado para o gol" (`FUT_OBJETIVO=gol`, padrão)

Código em `strategy/src/GoalApproach.cpp`. Com `FUT_OBJETIVO=bola` o robô volta
ao modo antigo (só ir até a bola e apontar para ela).

| Parâmetro | Valor | Função |
|---|---|---|
| Gol de ataque | centro em (+1,185 ; 0) m, boca de 0,52 m | O da direita da imagem |
| `BEHIND_BALL_M` | 0,20 m (= `STOP_DISTANCE_M`) | Ponto alvo: atrás da bola, na linha gol → bola |
| `FIELD_MARGIN_M` | 0,12 m | O ponto alvo fica pelo menos isso longe da borda |
| Lei de controle | polar (Siegwart; Astolfi): v = K_RHO·ρ, w = K_ALPHA·α + K_BETA·β | Estável com K_RHO > 0, K_BETA < 0, K_ALPHA − K_RHO > 0 (checado em `static_assert`) |
| `K_RHO` / `K_ALPHA` / `K_BETA` | 0,4 / 0,7 / −0,15 (1/s) | K_ALPHA = o mesmo ganho de orientação sintonizado pelo atraso |
| `POSE_REACHED_M` / `POSE_LEFT_M` | 0,04 / 0,08 m | Chegou no ponto: só gira no lugar até ficar virado para o gol (histerese) |
| `BALL_CLEARANCE_M` / `BALL_CLEARANCE_EXIT_M` | 0,16 / 0,22 m | Folga mínima do caminho até a bola: abaixo disso, contorna |
| `AVOID_SIDE_M` / `AVOID_BEHIND_M` | 0,30 / 0,30 m | Pontos de contorno: ao lado da bola e depois ao lado e atrás |
| `AVOID_ADVANCE_CONE` | 0,25 rad | Contornando, só anda quando já aponta para o ponto de contorno |

Na simulação (atraso de 0,37 s, velocidade mínima das rodas, posição em cm
inteiros), partindo de 10 posições diferentes, inclusive na frente da bola, o
robô chegou a ~3 cm do ponto alvo e a até ~6° do gol, sem chegar a menos de
20 cm do centro da bola (exceto começando colado nela: 16,9 cm).

---

## 8. Como medir de novo

Com a `vision` rodando e o robô no centro do campo, sem a bola:

```bash
cd FUT-ROBO/strategy
cmake --build build

# Calibração dos motores (~3 min): partida, parada, curva PWM x velocidade,
# comparação entre as rodas e desvio da tag
FUT_TESTE_MOTOR=1 ./build/strategy

# Teste de giro (~42 s): confere se o robô gira na velocidade pedida
FUT_TESTE_GIRO=1 ./build/strategy
```

- Ctrl+C em qualquer teste manda o robô parar antes de sair.
- Depois da calibração dos motores, copie a linha "média" da curva para
  `CAL_SPEED` / `CAL_PWM` no firmware e para `FW_CAL_SPEED` / `FW_CAL_PWM` no
  `strategy`, e o último valor de velocidade para `MAX_WHEEL_SPEED_MS`.
- Se mudar a bitola, atualize `TRACK_WIDTH` (firmware) e `TRACK_WIDTH_M` (strategy).
