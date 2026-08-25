// Firmware do braço robótico — Arduino Nano + módulo PCA9685 (I2C).
//
// Fala o MESMO protocolo de texto que o MockArmSerial do dashboard
// (src/arm/mock_arm_serial.py), então o lado Python não muda nada além
// de detectar a porta serial e encaminhar as linhas de comando pra cá:
//
//   SET <id> <angulo>   -> "OK SET <id> <angulo>"
//   GET <id>            -> "POS <id> <angulo>"
//   GET ALL             -> "POS ALL 0:<a0> 1:<a1> ..."
//   HOME                -> todos os servos pra 90 graus, responde "OK HOME"
//   <comando inválido>  -> "ERR ..."
//
// Ligação:
//   PCA9685 VCC -> 5V do Nano (lógica) | PCA9685 V+ -> fonte externa dos servos
//   PCA9685 GND -> GND do Nano (comum com a fonte externa)
//   PCA9685 SDA -> A4 do Nano | PCA9685 SCL -> A5 do Nano
//   Servos: base=canal 0, ombro=canal 1, cotovelo=canal 2, garra=canal 3
//   (ajuste CANAL_PCA abaixo se a fiação real for outra)
//
// Biblioteca necessária (Arduino IDE -> Gerenciador de Bibliotecas):
//   "Adafruit PWM Servo Driver Library"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_SERVOS = 4;  // base, ombro, cotovelo, garra
const uint8_t CANAL_PCA[NUM_SERVOS] = {0, 1, 2, 3};

// Calibração do pulso (em contagens de 0-4095 a 50Hz). Os valores abaixo são
// um ponto de partida comum para servos analógicos de 180°; ajuste se algum
// servo não bater exatamente 0°/180° na prática.
const int SERVOMIN = 150;  // ~0 graus
const int SERVOMAX = 600;  // ~180 graus
const int PWM_FREQ = 50;

float angulos[NUM_SERVOS];
String bufferSerial;

void moverServo(uint8_t id, float angulo) {
  angulo = constrain(angulo, 0, 180);
  angulos[id] = angulo;
  int pulso = map((long)angulo, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(CANAL_PCA[id], 0, pulso);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    moverServo(i, 90);
  }
}

void processaComando(String linha) {
  linha.trim();
  if (linha.length() == 0) return;

  String cmdLinha = linha;
  cmdLinha.toUpperCase();

  int p1 = cmdLinha.indexOf(' ');
  String cmd = (p1 == -1) ? cmdLinha : cmdLinha.substring(0, p1);
  String resto = (p1 == -1) ? "" : cmdLinha.substring(p1 + 1);

  if (cmd == "SET") {
    int p2 = resto.indexOf(' ');
    if (p2 == -1) {
      Serial.println("ERR BAD_ARGS " + linha);
      return;
    }
    int id = resto.substring(0, p2).toInt();
    float angulo = resto.substring(p2 + 1).toFloat();
    if (id < 0 || id >= NUM_SERVOS) {
      Serial.println("ERR BAD_ARGS " + linha);
      return;
    }
    moverServo(id, angulo);
    Serial.println("OK SET " + String(id) + " " + String(angulos[id], 1));

  } else if (cmd == "HOME") {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      moverServo(i, 90);
    }
    Serial.println("OK HOME");

  } else if (cmd == "GET" && resto == "ALL") {
    String out = "POS ALL";
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      out += " " + String(i) + ":" + String(angulos[i], 1);
    }
    Serial.println(out);

  } else if (cmd == "GET") {
    int id = resto.toInt();
    if (id < 0 || id >= NUM_SERVOS) {
      Serial.println("ERR BAD_ARGS " + linha);
      return;
    }
    Serial.println("POS " + String(id) + " " + String(angulos[id], 1));

  } else {
    Serial.println("ERR UNKNOWN " + linha);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processaComando(bufferSerial);
      bufferSerial = "";
    } else if (c != '\r') {
      bufferSerial += c;
    }
  }
}
