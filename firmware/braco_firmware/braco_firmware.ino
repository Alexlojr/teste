// Firmware do braço robótico — Arduino Nano + módulo PCA9685 (I2C).
//
// Sketch único: faz a operação normal (comandos vindos da Raspberry Pi) E os
// testes de bancada da montagem. Ter dois sketches separados significava
// trocar de firmware no meio da calibração e lembrar de trocar o baudrate
// junto — quando não se lembra, a serial devolve lixo e parece defeito de
// hardware. Aqui não há o que trocar.
//
// ---------------------------------------------------------------------------
// LIGAÇÃO
// ---------------------------------------------------------------------------
//   PCA9685 VCC -> 5V do Nano (lógica)  |  PCA9685 V+ -> fonte externa 5V/10A
//   PCA9685 GND -> GND do Nano, comum com o GND da fonte externa
//   PCA9685 SDA -> A4 do Nano           |  PCA9685 SCL -> A5 do Nano
//   Servos: base=canal 0, ombro=canal 1, cotovelo=canal 2, garra=canal 3
//
// Biblioteca: "Adafruit PWM Servo Driver Library" (Gerenciador de Bibliotecas)
//
// ---------------------------------------------------------------------------
// PROTOCOLO — texto, uma linha por comando, 115200 baud
// ---------------------------------------------------------------------------
// Operação (o lado Python só usa estes):
//   SET <id> <ang>        move a junta para o ângulo, com movimento suavizado
//   GET <id> | GET ALL    posição atual (durante o movimento, a intermediária)
//   HOME                  volta todas para a pose de repouso
//   BUSY                  "BUSY 1" enquanto alguma junta se move, senão "BUSY 0"
//   STOP                  para tudo onde está, agora
//
// Bancada / calibração (passo 2 do roteiro):
//   SWEEP <id> <de> <ate> vai e volta na janela, devagar, até um STOP
//   LIM                   mostra os limites de todas as juntas
//   LIM <id> <min> <max>  redefine o limite de uma junta
//   SPEED <graus_s>       velocidade média (padrão 60 °/s)
//   DETACH <id>|ALL       corta o PWM: dá pra mexer a junta com a mão
//   ATTACH <id>|ALL       religa o PWM na posição atual
//   RAW <id> <pulso>      escreve a contagem de pulso crua (150-600 típico)
//   PULSO <id> <min> <max>  guarda a calibração de pulso achada com RAW
//   HELP                  lista os comandos
//
// Erros respondem "ERR <motivo> <linha>". Todo comando responde alguma coisa:
// silêncio significa que a linha não chegou.
//
// ---------------------------------------------------------------------------
// SEGURANÇA DA CALIBRAÇÃO
// ---------------------------------------------------------------------------
// Use SWEEP em janelas de ~20°, nunca de ponta a ponta. Se a junta bater num
// limite mecânico no meio do curso, o servo trava e continua puxando corrente
// — é assim que se descasca engrenagem de MG90S. Com STOP a um comando de
// distância, dá pra cortar antes de estragar.

#include <stdlib.h>          // atoi(), atof() — ver a nota sobre sscanf abaixo
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_SERVOS = 4;
const uint8_t CANAL_PCA[NUM_SERVOS] = {0, 1, 2, 3};

const int PWM_FREQ = 50;
const unsigned long INTERVALO_PASSO_MS = 20;   // 50 Hz de atualização

// Calibração de pulso, por servo (contagens de 0-4095 a 50 Hz). O valor comum
// para servos analógicos de 180° é 150-600, mas cada peça varia. Ache o real
// com RAW e guarde com PULSO.
int pulso_min[NUM_SERVOS] = {150, 150, 150, 150};
int pulso_max[NUM_SERVOS] = {600, 600, 600, 600};

// Limites por junta, em graus de servo.
//
// ATENÇÃO: estes valores duplicam os de config/braco.json e começam iguais aos
// placeholders de lá. Duplicata é risco de dessincronizar, e a saída para isso
// é o comando LIM: o lado Python pode empurrar os limites do JSON ao conectar,
// fazendo do braco.json a fonte única de verdade. Até isso existir, mantenha
// os dois arquivos iguais na mão.
//
// A proteção do firmware não é redundante: um SET digitado à mão no terminal
// não passa por nenhuma checagem do lado Python.
float lim_min[NUM_SERVOS] = { 10.0,  20.0,  10.0,  30.0};
float lim_max[NUM_SERVOS] = {170.0, 160.0, 170.0, 130.0};

// Pose de repouso do HOME. Recalcule depois da calibração: 90° em todas as
// juntas é chute, não necessariamente uma pose estável para este braço.
float pose_repouso[NUM_SERVOS] = {90.0, 90.0, 90.0, 90.0};

float atual[NUM_SERVOS];      // posição sendo escrita agora (segue a curva)
float alvo[NUM_SERVOS];       // para onde o movimento está indo
float origem[NUM_SERVOS];     // de onde partiu — a curva interpola origem->alvo
bool  ligado[NUM_SERVOS];     // false = PWM cortado, junta solta

unsigned long t_inicio[NUM_SERVOS];   // quando este movimento começou
unsigned long duracao[NUM_SERVOS];    // quanto ele deve durar, em ms

float velocidade = 60.0;      // graus por segundo, MÉDIA (ver suaviza())

// Um movimento curto demais fica com duração de poucos milissegundos e volta
// a ser degrau. Este piso garante que até um ajuste de 1° seja suavizado.
const unsigned long DURACAO_MIN_MS = 80;

// Movimentos disparados dentro desta janela contam como um lote só e recebem
// todos a MESMA duração. A sequência de coleta manda "SET 0 / SET 1 / SET 2"
// em linhas seguidas, com microssegundos entre elas: sem isso, cada junta
// chegaria num instante diferente e a garra faria um caminho torto no ar,
// diferente do que a cinemática planejou.
const unsigned long JANELA_LOTE_MS = 50;
unsigned long t_lote = 0;
unsigned long duracao_lote = 0;
bool no_lote[NUM_SERVOS];

// Estado do SWEEP. Fica no laco principal em vez de num for() bloqueante:
// bloquear significaria não conseguir ler o STOP enquanto a junta se move,
// que é justamente quando ele é necessário.
bool    sweep_ativo = false;
uint8_t sweep_id = 0;
float   sweep_de = 0, sweep_ate = 0;

unsigned long t_ultimo_passo = 0;

char linha[48];
uint8_t n_linha = 0;

// ---------------------------------------------------------------------------
// Saída para os servos
// ---------------------------------------------------------------------------

// Converte ângulo em contagem de pulso SEM truncar para inteiro no caminho.
// O firmware antigo fazia map((long)angulo, ...), o que jogava fora a casa
// decimal que a cinemática inversa calcula. Com ~160 mm de alcance, 1° vale
// cerca de 2,8 mm na ponta da garra — arredondar custava até 1,4 mm.
void escrevePulso(uint8_t id, float angulo) {
  float fracao = angulo / 180.0;
  long pulso = lround(pulso_min[id] + fracao * (pulso_max[id] - pulso_min[id]));
  pulso = constrain(pulso, 0, 4095);
  pwm.setPWM(CANAL_PCA[id], 0, (uint16_t)pulso);
}

void desligaSaida(uint8_t id) {
  pwm.setPWM(CANAL_PCA[id], 0, 4096);   // bit de "full off" do PCA9685
}

float limitaAoIntervalo(uint8_t id, float angulo) {
  return constrain(angulo, lim_min[id], lim_max[id]);
}

// ---------------------------------------------------------------------------
// Suavização do movimento
// ---------------------------------------------------------------------------
// O problema que isto resolve são os trancos.
//
// Mandar o ângulo final de uma vez faz o servo sair na velocidade máxima dele
// e bater no fim do curso: tranco na partida e na chegada. Interpolar em
// linha reta (um for com delay, ou um passo fixo por ciclo) melhora o meio do
// caminho mas NÃO resolve as pontas — a velocidade ainda salta de zero para o
// valor cheio e volta a zero de repente. Aceleração instantânea é o tranco.
//
// A saída é interpolar a POSIÇÃO por uma curva cuja derivada é zero nas duas
// pontas. Assim o servo parte do repouso e chega ao repouso, sem degrau de
// velocidade em lugar nenhum.
//
// suaviza() é a smootherstep: 6p^5 - 15p^4 + 10p^3. Sua primeira E segunda
// derivadas são zero em p=0 e p=1, então nem a velocidade nem a aceleração
// dão salto. É por isso que ela some com o tranco também no retorno e no fim
// do sweep, que é onde a interpolação linear ainda trancava.
//
// Custo: para uma mesma duração, o pico de velocidade fica ~1,9x a média.
// `velocidade` (SPEED) é a MÉDIA — "SPEED 60" significa que um movimento de
// 60° leva 1 segundo, não que o servo nunca passa de 60°/s.

float suaviza(float p) {
  if (p <= 0.0) return 0.0;
  if (p >= 1.0) return 1.0;
  return p * p * p * (p * (p * 6.0 - 15.0) + 10.0);
}

void defineAlvo(uint8_t id, float angulo) {
  angulo = limitaAoIntervalo(id, angulo);
  unsigned long agora = millis();

  // Abre um lote novo se o anterior já esfriou. Movimentos disparados juntos
  // compartilham a duração para chegarem juntos.
  if (agora - t_lote > JANELA_LOTE_MS) {
    t_lote = agora;
    duracao_lote = 0;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) no_lote[i] = false;
  }

  // Partir da posição ATUAL, não do alvo anterior: se um SET novo chega no
  // meio de um movimento, continuar de onde o servo realmente está é o que
  // evita o salto de posição — que seria outro tranco.
  origem[id] = atual[id];
  alvo[id] = angulo;
  no_lote[id] = true;

  unsigned long natural = (unsigned long)(fabs(angulo - origem[id]) / velocidade * 1000.0);
  if (natural < DURACAO_MIN_MS) natural = DURACAO_MIN_MS;
  if (natural > duracao_lote) duracao_lote = natural;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (no_lote[i]) {
      t_inicio[i] = t_lote;
      duracao[i] = duracao_lote;
    }
  }
}

bool algumaEmMovimento() {
  unsigned long agora = millis();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (ligado[i] && agora - t_inicio[i] < duracao[i]) return true;
  }
  return false;
}

void paraTudo() {
  sweep_ativo = false;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    // Congela onde está: origem = alvo = posição atual, duração zerada.
    origem[i] = atual[i];
    alvo[i] = atual[i];
    duracao[i] = 0;
  }
}

void atualizaMovimento() {
  unsigned long agora = millis();
  if (agora - t_ultimo_passo < INTERVALO_PASSO_MS) return;
  t_ultimo_passo = agora;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!ligado[i]) continue;

    float posicao;
    if (duracao[i] == 0) {
      posicao = alvo[i];
    } else {
      unsigned long decorrido = agora - t_inicio[i];
      if (decorrido >= duracao[i]) {
        posicao = alvo[i];
        duracao[i] = 0;          // movimento encerrado
      } else {
        float p = (float)decorrido / (float)duracao[i];
        posicao = origem[i] + (alvo[i] - origem[i]) * suaviza(p);
      }
    }

    // Não reescreve o pulso se nada mudou: com o braço parado isso zera o
    // tráfego I2C em vez de repetir o mesmo valor 50 vezes por segundo.
    if (fabs(posicao - atual[i]) < 0.01) continue;

    atual[i] = posicao;
    escrevePulso(i, atual[i]);
  }

  // O sweep só inverte quando a junta chegou de fato. Como cada trecho é uma
  // curva suavizada inteira, a inversão acontece com o servo já parado — que
  // é justamente onde o sweep antigo dava o tranco mais feio.
  if (sweep_ativo && duracao[sweep_id] == 0) {
    defineAlvo(sweep_id, (fabs(alvo[sweep_id] - sweep_de) < 0.05) ? sweep_ate : sweep_de);
  }
}

// ---------------------------------------------------------------------------
// Comandos
// ---------------------------------------------------------------------------

bool idValido(int id) {
  return id >= 0 && id < NUM_SERVOS;
}

void respondePos(uint8_t id) {
  Serial.print(F("POS "));
  Serial.print(id);
  Serial.print(' ');
  Serial.println(atual[id], 1);
}

void respondeLimites() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("LIM "));
    Serial.print(i);
    Serial.print(' ');
    Serial.print(lim_min[i], 1);
    Serial.print(' ');
    Serial.println(lim_max[i], 1);
  }
}

void mostraAjuda() {
  Serial.println(F("SET <id> <ang> | GET <id>|ALL | HOME | BUSY | STOP"));
  Serial.println(F("SWEEP <id> <de> <ate> | LIM [<id> <min> <max>] | SPEED <g/s>"));
  Serial.println(F("DETACH <id>|ALL | ATTACH <id>|ALL | RAW <id> <pulso>"));
  Serial.println(F("PULSO <id> <min> <max> | HELP"));
}

void erroArgs(const char *linha) {
  Serial.print(F("ERR BAD_ARGS "));
  Serial.println(linha);
}


// Divide a linha em tokens separados por espaço ou tabulação, terminando cada
// um com '\0' no próprio buffer. Devolve quantos tokens saíram.
uint8_t divideEmTokens(char *texto, char *tok[], uint8_t maximo) {
  uint8_t n = 0;
  char *p = texto;

  while (*p && n < maximo) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') break;
    tok[n++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = '\0';
  }
  return n;
}

// ---------------------------------------------------------------------------
// Interpretação dos comandos
// ---------------------------------------------------------------------------
// Os argumentos são convertidos com atoi() e atof(), e NÃO com sscanf().
//
// O motivo é uma armadilha do avr-libc: a biblioteca padrão do Arduino linka
// uma versão reduzida do scanf que não implementa ponto flutuante. Um
// sscanf(texto, "SET %d %f", ...) compila sem aviso, roda, e simplesmente
// falha em converter o segundo argumento, devolvendo 1 em vez de 2. O comando
// então cai no final da cadeia e responde ERR UNKNOWN, como se a linha tivesse
// sido digitada errada. É a mesma razão pela qual sprintf("%f") não funciona
// no Arduino e existe o dtostrf.
//
// atof() faz parte do avr-libc e funciona normalmente.

void processaComando(char *texto) {
  for (char *p = texto; *p; p++) *p = toupper(*p);

  char *tok[4];
  uint8_t n = divideEmTokens(texto, tok, 4);
  if (n == 0) return;

  const char *cmd = tok[0];

  // ---- operação ---------------------------------------------------------

  if (strcmp(cmd, "SET") == 0 && n == 3) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    sweep_ativo = false;              // um SET manual cancela o sweep em curso
    defineAlvo(id, atof(tok[2]));
    Serial.print(F("OK SET "));
    Serial.print(id);
    Serial.print(' ');
    Serial.println(alvo[id], 1);

  } else if (strcmp(cmd, "GET") == 0 && n == 2 && strcmp(tok[1], "ALL") == 0) {
    Serial.print(F("POS ALL"));
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(' ');
      Serial.print(i);
      Serial.print(':');
      Serial.print(atual[i], 1);
    }
    Serial.println();

  } else if (strcmp(cmd, "GET") == 0 && n == 2) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    respondePos(id);

  } else if (strcmp(cmd, "HOME") == 0 && n == 1) {
    sweep_ativo = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) defineAlvo(i, pose_repouso[i]);
    Serial.println(F("OK HOME"));

  } else if (strcmp(cmd, "STOP") == 0 && n == 1) {
    paraTudo();
    Serial.println(F("OK STOP"));

  } else if (strcmp(cmd, "BUSY") == 0 && n == 1) {
    Serial.print(F("BUSY "));
    Serial.println(algumaEmMovimento() ? 1 : 0);

  // ---- bancada ----------------------------------------------------------

  } else if (strcmp(cmd, "SWEEP") == 0 && n == 4) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    sweep_de  = limitaAoIntervalo(id, atof(tok[2]));
    sweep_ate = limitaAoIntervalo(id, atof(tok[3]));
    sweep_id = id;
    sweep_ativo = true;
    defineAlvo(id, sweep_de);
    Serial.print(F("OK SWEEP "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(sweep_de, 1);
    Serial.print(' ');
    Serial.println(sweep_ate, 1);

  } else if (strcmp(cmd, "LIM") == 0 && n == 4) {
    int id = atoi(tok[1]);
    float a = atof(tok[2]);
    float b = atof(tok[3]);
    if (!idValido(id) || a > b) { erroArgs(texto); return; }
    lim_min[id] = a;
    lim_max[id] = b;
    defineAlvo(id, atual[id]);        // realinha o alvo se o novo limite o excluiu
    Serial.print(F("OK LIM "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(a, 1);
    Serial.print(' ');
    Serial.println(b, 1);

  } else if (strcmp(cmd, "LIM") == 0 && n == 1) {
    respondeLimites();

  } else if (strcmp(cmd, "SPEED") == 0 && n == 2) {
    float a = atof(tok[1]);
    // Teto de 150 °/s de MÉDIA porque a curva suavizada tem pico de 1,875x a
    // média: 150 vira 281 °/s no meio do movimento, que já é bastante para um
    // MG90S com carga. Acima disso o servo satura, não acompanha a curva, e a
    // suavização deixa de valer — o tranco volta.
    if (a <= 0 || a > 150) { erroArgs(texto); return; }
    velocidade = a;
    Serial.print(F("OK SPEED "));
    Serial.println(velocidade, 1);

  } else if (strcmp(cmd, "DETACH") == 0 && n == 2 && strcmp(tok[1], "ALL") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) { ligado[i] = false; desligaSaida(i); }
    sweep_ativo = false;
    Serial.println(F("OK DETACH ALL"));

  } else if (strcmp(cmd, "DETACH") == 0 && n == 2) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    ligado[id] = false;
    desligaSaida(id);
    if (sweep_ativo && sweep_id == id) sweep_ativo = false;
    Serial.print(F("OK DETACH "));
    Serial.println(id);

  } else if (strcmp(cmd, "ATTACH") == 0 && n == 2 && strcmp(tok[1], "ALL") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      ligado[i] = true;
      alvo[i] = atual[i];
      duracao[i] = 0;
      escrevePulso(i, atual[i]);
    }
    Serial.println(F("OK ATTACH ALL"));

  } else if (strcmp(cmd, "ATTACH") == 0 && n == 2) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    ligado[id] = true;
    alvo[id] = atual[id];             // religa onde está, não onde estava antes
    duracao[id] = 0;
    escrevePulso(id, atual[id]);
    Serial.print(F("OK ATTACH "));
    Serial.println(id);

  } else if (strcmp(cmd, "RAW") == 0 && n == 3) {
    int id = atoi(tok[1]);
    int pulso = atoi(tok[2]);
    if (!idValido(id) || pulso < 0 || pulso > 4095) { erroArgs(texto); return; }
    sweep_ativo = false;
    pwm.setPWM(CANAL_PCA[id], 0, (uint16_t)pulso);
    Serial.print(F("OK RAW "));
    Serial.print(id);
    Serial.print(' ');
    Serial.println(pulso);

  } else if (strcmp(cmd, "PULSO") == 0 && n == 4) {
    int id = atoi(tok[1]);
    int pi = atoi(tok[2]);
    int pj = atoi(tok[3]);
    if (!idValido(id) || pi < 0 || pj > 4095 || pi >= pj) { erroArgs(texto); return; }
    pulso_min[id] = pi;
    pulso_max[id] = pj;
    Serial.print(F("OK PULSO "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(pi);
    Serial.print(' ');
    Serial.println(pj);

  } else if (strcmp(cmd, "HELP") == 0 && n == 1) {
    mostraAjuda();

  } else {
    Serial.print(F("ERR UNKNOWN "));
    Serial.println(cmd);
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // Assume a pose de repouso como posição inicial em vez de ir até ela:
  // no boot não se sabe onde as juntas estão, e ramper a partir de um palpite
  // errado seria um movimento não comandado logo ao ligar.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    atual[i] = pose_repouso[i];
    alvo[i] = pose_repouso[i];
    ligado[i] = true;
    escrevePulso(i, atual[i]);
  }

  t_ultimo_passo = millis();
  Serial.println(F("READY braco_firmware"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      linha[n_linha] = '\0';
      if (n_linha > 0) processaComando(linha);
      n_linha = 0;
    } else if (c != '\r') {
      // Linha longa demais é descartada inteira em vez de truncada: metade de
      // um comando de ângulo é pior do que comando nenhum.
      if (n_linha < sizeof(linha) - 1) {
        linha[n_linha++] = c;
      } else {
        n_linha = 0;
        Serial.println(F("ERR TOO_LONG"));
      }
    }
  }

  atualizaMovimento();
}
