#include <SimpleKalmanFilter.h>

#define TRIG_PIN 4
#define ECHO_PIN 3

long duration;

SimpleKalmanFilter kalmanFilter(2, 2, 0.01); // Kalman

uint16_t readHCSR04_KalmanMM();
void UART_SendDistance();

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

uint16_t readHCSR04_KalmanMM() {
  static float filteredMM = 0.0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 20000); // timeout 20ms
  if (duration == 0) return 0;

  float distanceMM = duration * 0.343 / 2.0;
  if (distanceMM < 20 || distanceMM > 4000) return 0;

  // Kalman filter
  filteredMM = kalmanFilter.updateEstimate(distanceMM);

  return (uint16_t)(filteredMM); //mm
}

void UART_SendDistance(uint16_t msg) {
  uint8_t TxData[4];

  TxData[0] = 0x0A;                        // header
  TxData[1] = (msg >> 8) & 0xFF;    // high byte
  TxData[2] = msg & 0xFF;           // low byte
  TxData[3] = TxData[0] ^ TxData[1] ^ TxData[2];  // CRC XOR

  Serial.write(TxData, 4); 
}

void loop() {
  uint16_t d = readHCSR04_KalmanMM();
  UART_SendDistance(d);

  delay(75); // ~13Hz
}