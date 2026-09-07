# Braço Separador Inteligente

Braço robótico que detecta objetos por visão computacional e os separa
automaticamente. TCC de Engenharia da Computação — UniSALESIANO Araçatuba.

Arquitetura mestre-escravo: a **Raspberry Pi 5** roda o YOLO, calcula as
coordenadas e serve a dashboard; o **Arduino Nano** recebe comandos por serial
e aciona os servos via PCA9685.

## Estrutura

```
app.py                        dashboard Streamlit: câmera ao vivo + YOLO
coletar.py                    pipeline de coleta ponta a ponta (headless)
calibrar_camera.py            gera config/homografia.json a partir de 4 pontos
mapa_alcance.py               desenha em ASCII o que o braço alcança
setup.sh                      setup do Raspberry Pi

config/
  braco.json                  medidas e calibração — o arquivo do dia a dia
  homografia.json             gerado pela calibração (não versionado)

firmware/
  braco_firmware/             sketch único: operação + testes de bancada

src/
  arm/
    kinematics.py             cinemática inversa + tradução para ângulo de servo
    coleta.py                 sequência de pega
    mock_arm_serial.py        simula os servos; encaminha para o Arduino se houver
  vision/
    deteccao.py               escolhe e desenha a detecção do YOLO
    homografia.py             pixel -> mm
    estabilidade.py           decide quando a coordenada pode ser usada
  camera.py                   visualizador standalone, sem Streamlit

tests/
  test_firmware.py            dispatch e perfil de velocidade, sem Arduino
  test_estabilidade.py        portão de estabilidade
  test_pipeline.py            corrente visão -> braço, sem câmera
```

Rodar os testes (nenhum precisa de hardware):

```bash
python tests/test_firmware.py && python tests/test_estabilidade.py && python tests/test_pipeline.py
```

## As camadas, e por que estão separadas

**`Braco`** (em `kinematics.py`) só conhece geometria: comprimento dos elos e
cinemática. Não sabe o que é servo.

**`CalibracaoServo`** só conhece a montagem: offset, sentido de giro e limites
físicos de cada junta. Não sabe o que é cinemática.

Se o braço for remontado ou trocado, **só a segunda camada muda**. É por isso
que nenhuma dimensão de braço aparece hardcoded em lugar nenhum do código:
tudo vem do `config/braco.json`.

## Convenção angular

Mantenha idêntica em qualquer porte deste código:

- `theta_base` — rotação no plano XY, 0° = eixo +X
- `theta_ombro` — ângulo do elo L2 em relação à horizontal
- `theta_cotovelo` — ângulo do elo L3 **relativo** ao L2 (0° = esticado)

Nas coordenadas da mesa, a base do braço é a origem: **+X** para a direita,
**+Y** para a frente.

## Protocolo serial

Texto, uma linha por comando, **115200 baud**. O `MockArmSerial` e o
`braco_firmware.ino` falam o mesmo protocolo, então trocar simulação por
hardware real não muda nada acima da serial.

Operação — é só isto que o lado Python usa:

```
SET <canal> <ângulo>   ->  OK SET <canal> <ângulo>
GET <canal>            ->  POS <canal> <ângulo>
GET ALL                ->  POS ALL 0:<a0> 1:<a1> ...
HOME                   ->  OK HOME
BUSY                   ->  BUSY 0 | BUSY 1
STOP                   ->  OK STOP
<inválido>             ->  ERR ...
```

Bancada — existem para a calibração do passo 2:

```
SWEEP <id> <de> <ate>    vai e volta na janela até um STOP
LIM                      mostra os limites de todas as juntas
LIM <id> <min> <max>     redefine o limite de uma junta
SPEED <graus_s>          velocidade média, 1–150 (padrão 60 °/s)
DETACH <id>|ALL          corta o PWM: dá para mexer a junta com a mão
ATTACH <id>|ALL          religa o PWM na posição atual
RAW <id> <pulso>         escreve a contagem de pulso crua
PULSO <id> <min> <max>   guarda a calibração de pulso achada com RAW
HELP                     lista os comandos
```

Canais: `0` base, `1` ombro, `2` cotovelo, `3` garra.

**Sketch único, de propósito.** Antes havia um firmware de produção e um de
bancada, com baudrates diferentes — trocar de sketch e esquecer disso faz a
serial devolver lixo e parecer defeito de hardware. Agora não há o que trocar.

## Sincronia entre o Python e o braço

O firmware suaviza os movimentos, então a duração deixou de ser fixa: fechar a
garra (100°) leva quase o triplo de descer até o objeto (33°). Por isso a
sequência de coleta **pergunta ao firmware quando ele parou** (`BUSY`) em vez
de esperar um tempo fixo.

Isso depende de o lado Python conseguir ouvir o Arduino, o que antes não
acontecia: o `readline()` do `MockArmSerial` devolvia sempre a resposta
simulada mesmo com hardware conectado, então um `ERR` do firmware nunca
aparecia. Agora, havendo Arduino, a resposta dele é a que vale.

`pausa_s` no `braco.json` deixou de ser a espera do movimento e passou a ser
só o tempo de assentamento depois da chegada — a estrutura de acrílico ainda
oscila um pouco quando o servo para.

## Movimento suavizado

Todo `SET` é interpolado por uma curva *smootherstep* (`6p⁵ − 15p⁴ + 10p³`),
cuja primeira e segunda derivadas são zero nas duas pontas. Na prática: o
servo **parte do repouso e chega ao repouso**, sem degrau de velocidade.

Isso existe porque interpolar em linha reta — um `for` com `delay`, ou um
passo fixo por ciclo — suaviza o meio do movimento mas não as pontas. A
velocidade ainda salta de zero para o valor cheio na partida e cai a zero na
chegada, e aceleração instantânea é exatamente o que se sente como tranco.
É por isso que o tranco aparecia principalmente no retorno e no fim do sweep.

Duas consequências que valem saber:

- **`SPEED` é a velocidade MÉDIA.** `SPEED 60` quer dizer que um movimento de
  60° leva 1 segundo. O pico no meio do trajeto é 1,875× a média — por isso o
  teto é 150 °/s: acima disso o servo satura, não acompanha a curva, e a
  suavização deixa de valer.
- **Movimentos disparados juntos chegam juntos.** `SET`s recebidos dentro de
  50 ms contam como um lote e recebem todos a mesma duração. Sem isso, cada
  junta chegaria num instante diferente e a garra faria no ar um caminho
  diferente do que a cinemática planejou.

`STOP` é a exceção deliberada: ele congela na hora, sem desacelerar. É freio
de emergência, e desacelerar suave numa emergência não é o que se quer.

## Ordem de montagem

Cada etapa depende da anterior estar certa. Pular uma faz a seguinte falhar de
um jeito difícil de diagnosticar.

**1. Medir os elos** → `config/braco.json`, campo `elos_mm`

De centro de eixo a centro de eixo:

- `l1` — altura **vertical** da superfície até o eixo do ombro
- `l2` — eixo do ombro → eixo do cotovelo (distância direta)
- `l3` — eixo do cotovelo → ponto onde a garra fecha

Se o eixo do ombro não fica exatamente acima do eixo da base, meça esse
deslocamento horizontal e ponha em `offset_radial_mm`. No CAD apareceu algo em
torno de 21 mm — confirmar no braço real.

**2. Achar os limites das juntas** → `config/braco.json`, campo `juntas`

Com o `braco_firmware` carregado, junta por junta. Baixe a velocidade antes
(`SPEED 10`) e deixe o `STOP` à mão:

- `minimo`/`maximo` — use `SWEEP <id> <de> <ate>` em janelas de 20°, **nunca**
  de ponta a ponta. Se a junta bater num limite mecânico no meio do curso, o
  servo trava e continua puxando corrente. É assim que se descasca engrenagem
  de MG90S. Ao ouvir o servo forçar, mande `STOP`.
- `DETACH <id>` solta a junta para você achar o curso com a mão antes de
  acionar o servo — é o jeito mais seguro de começar.
- `offset` — que ângulo de servo deixa o elo no 0° matemático
- `sentido` — mande 90, depois 110; se o elo foi para o lado contrário do
  esperado, use `-1`

Neste braço o cotovelo é acionado por paralelogramo, então o ângulo dele tende
a ser medido em relação à horizontal, não ao elo do ombro. Isso costuma
aparecer como um `offset`/`sentido` contra-intuitivo — é esperado, não é erro
de medida.

Ao terminar, marque `"calibrado": true`.

**3. Conferir o alcance** → `python mapa_alcance.py`

Rode antes de fixar a área de coleta em definitivo. Os símbolos importam:

- muitos `x` (limite de junta) → limites apertados demais, ou área longe da base
- muitos `.` (fora do envelope) → o braço é curto para essa área

**4. Fixar a câmera e calibrar** → `python calibrar_camera.py`

4 pontos medidos em mm relativos à base do braço. Erro residual acima de ~5 mm
quer dizer ponto mal medido ou mal clicado. Vale enquanto câmera, mesa e base
não se moverem — se qualquer um sair do lugar, refaça.

**5. Dry-run** → `python coletar.py --dry-run`

Mostra as coordenadas detectadas e os comandos que sairiam, sem abrir a
serial. Confira se os milímetros batem com onde o objeto está de verdade
antes de energizar os servos.

**6. Primeira coleta real**, com a mão no interruptor da fonte.

**7. Treinar o YOLO** nos objetos do projeto. Por último de propósito: é a
etapa mais demorada, e detecção perfeita não adianta se o braço não alcança.

## A dashboard

Só visão: câmera ao vivo com as bounding boxes desenhadas e contagem dos
objetos que passam. Ainda usa classes COCO (0 = pessoa, 67 = celular) até o
modelo ser treinado nos objetos do projeto.

**A dashboard não move o braço.** Isso é deliberado: enquanto a maquete não
está montada e a homografia não está calibrada, acionar servo por interface
gráfica só esconde o que está acontecendo. Durante a montagem, use as
ferramentas de linha de comando:

```
python src/arm/mock_arm_serial.py    terminal interativo: SET / GET / HOME
python mapa_alcance.py               o que o braço alcança, sem hardware
python calibrar_camera.py            gera a homografia
```

A coleta de verdade é o `coletar.py`, que roda sem interface.

## O pipeline de coleta

`python coletar.py` liga a corrente inteira:

```
captura frame
  -> YOLO detecta o objeto de maior confiança
  -> portão de estabilidade acumula detecções até o objeto parar
  -> homografia converte a base da caixa em milímetros
  -> cinemática inversa converte em ângulos de servo
  -> sequência de coleta manda os SET pela serial
  -> rearma e espera o próximo objeto
```

**Por que esperar o objeto parar.** Duas razões, e a segunda é a menos óbvia:
com o movimento suavizado o braço leva ~2 s para chegar, então um objeto ainda
escorregando seria agarrado onde estava e não onde está; e mesmo com o objeto
totalmente parado a caixa do YOLO treme alguns pixels de frame para frame, e
uma detecção isolada carrega esse ruído inteiro para dentro da coordenada.

O portão resolve os dois: guarda as últimas `janela` detecções e só libera
quando todas cabem dentro de `tolerancia_mm`. Se cabem, o objeto está parado
**e** a mediana delas é uma coordenada melhor que qualquer uma sozinha. Os
parâmetros ficam na seção `estabilidade` do `braco.json`.

A tolerância é medida em milímetros e não em pixels de propósito: a grandeza
que importa é física — "o objeto se mexeu menos que a folga da garra" — e o
mesmo número de pixels vale distâncias diferentes conforme a posição na mesa.

**`python coletar.py --dry-run`** mostra os comandos sem abrir a serial. É o
passo 5 do roteiro. O script se recusa a rodar sem `"calibrado": true` e sem
`config/homografia.json`, porque apontar o braço com placeholder é exatamente
como se descasca engrenagem.

## Decisões que valem saber

**A garra fica fora da cadeia de IK.** São 4 servos, mas só 3 definem a posição
da ponta. A garra só abre e fecha, então a IK resolve 3 incógnitas, não 4.

**Não há PID, e é intencional.** MG90S e SG90 têm controle de posição interno
(potenciômetro no eixo). O microcontrolador manda o ângulo-alvo; o controle
fino é do próprio servo. PID externo só faria sentido com motor DC cru.

**IK analítica por lei dos cossenos, não iterativa.** Com 3 juntas a solução
fechada é exata e mais barata que FABRIK ou Jacobiana. Escolhida a solução
"elbow-up".

**Os ângulos da garra são configuração, não dedução.** `garra_aberta_graus` e
`garra_fechada_graus` no `braco.json`. Deixá-los em `null` faz a coleta cair no
mínimo/máximo da junta, que é chute em dois sentidos: qual extremo fecha
depende de como a garra foi montada — montada ao contrário, ela abre na hora
de agarrar — e ir sempre até o limite mecânico ignora que as peças de teste
são impressas com região de preensão dimensionada para um curso específico.
Com *form closure* a retenção é geométrica; apertar além do encaixe só aquece
o servo.

**A coleta valida o plano inteiro antes de mover.** Se alguma pose da sequência
for impossível, nada é enviado — evita o braço parar no meio do trajeto com o
objeto preso na garra.

**A homografia usa a base da bounding box, não o centro.** A base é onde o
objeto encosta na mesa, que é o plano da calibração. Usar o centro erra cada
vez mais conforme o objeto é mais alto.

**Detecção estática, não rastreamento.** O objeto está parado quando é
detectado, então 5–8 FPS do YOLOv8n no Pi 5 bastam: uma inferência
bem-sucedida já dá as coordenadas. É o argumento que justifica o Raspberry Pi 5
em vez de uma Jetson.

## O que ainda não está resolvido

- **Os limites do firmware são um padrão de segurança, não a fonte.** O
  `coletar.py` empurra os valores do `braco.json` via `LIM` ao conectar, então
  o JSON manda. Mas se alguém mover o braço por fora do `coletar.py` (pelo
  terminal do `mock_arm_serial`, por exemplo), valem os valores compilados no
  sketch — vale mantê-los razoáveis.
- **O firmware não foi compilado nem carregado ainda.** A lógica de dispatch e
  a rampa foram testadas por simulação, não na Arduino IDE.
- **O modelo ainda usa classes COCO**, não os objetos do projeto.
- **O `coletar.py` nunca rodou com câmera de verdade.** A corrente abaixo da
  câmera é testada em `tests/test_pipeline.py`, mas os dois elos de cima
  (Picamera2 e YOLO) só existem no Pi.
- **Nada rodou com o braço físico ainda** — tudo foi testado com serial falsa.
