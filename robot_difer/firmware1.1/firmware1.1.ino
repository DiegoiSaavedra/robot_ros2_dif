#include <Arduino.h>

// ================== DEFINICIONES DE PINES ==================
// Motor Izquierdo (Motor 1)
const int pwmA1     = 5;    // Pin A del driver L298N (o similar)
const int pwmB1     = 4;    // Pin B del driver L298N (o similar)
const int encoderA1 = 12;   // Encoder fase A
const int encoderB1 = 13;   // Encoder fase B

// Motor Derecho (Motor 2)
const int pwmA2     = 2;    // Pin A del driver L298N (o similar)
const int pwmB2     = 3;    // Pin B del driver L298N (o similar)
const int encoderA2 = 9;    // Encoder fase A
const int encoderB2 = 8;    // Encoder fase B

// ================== CONSTANTES PARA CONTROL ==================
#define CONTROL_INTERVAL_MS 50  // Intervalo de control en ms

// Parámetros del encoder y del robot
const float COUNTS_PER_REVOLUTION = 4532.0;   // Pulsos por revolución
const float WHEEL_RADIUS = 0.035;                // Radio de la rueda en metros (ej. 3.5 cm)
const float WHEEL_CIRCUMFERENCE = 2.0 * PI * WHEEL_RADIUS; // Circunferencia en metros

// Parámetros del PID (ajusta estos valores en función de tus pruebas)
float Kp = 100.0;
float Ki = 50.0;
float Kd = 10.10;

// Límites de la señal PWM
const int MIN_PWM = 98;  // PWM mínimo para vencer la inercia/fricción estática
const int MAX_PWM = 250; // PWM máximo

// ================== VARIABLES GLOBALES PARA CONTROL DE VELOCIDAD ==================
volatile float desiredSpeedLeft = 0.0;
volatile float desiredSpeedRight = 0.0;
float measuredSpeedLeft = 0.0;
float measuredSpeedRight = 0.0;
float integralLeft = 0.0, lastErrorLeft = 0.0;
float integralRight = 0.0, lastErrorRight = 0.0;
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
volatile long lastEncoderCountLeft = 0;
volatile long lastEncoderCountRight = 0;
unsigned long lastControlTime = 0;
volatile bool motorsActive = false;

// ================== VARIABLES PARA LAS INTERRUPCIONES DE LOS ENCODERS ==================
volatile byte lastStateA1 = 0;
volatile byte lastStateB1 = 0;
volatile byte stateA1 = 0;
volatile byte stateB1 = 0;
volatile byte lastStateA2 = 0;
volatile byte lastStateB2 = 0;
volatile byte stateA2 = 0;
volatile byte stateB2 = 0;

// ================== ISR DE LOS ENCODERS ==================
// (Sin cambios aquí)
IRAM_ATTR void handleEncoderLeft() {
  noInterrupts();
  stateA1 = digitalRead(encoderA1);
  stateB1 = digitalRead(encoderB1);
  int change = 0;
  if (stateA1 != lastStateA1) { if (stateA1 != stateB1) change++; else change--; }
  if (stateB1 != lastStateB1) { if (stateB1 == stateA1) change++; else change--; }
  encoderCountLeft += change;
  lastStateA1 = stateA1; lastStateB1 = stateB1;
  interrupts();
}
IRAM_ATTR void handleEncoderRight() {
  noInterrupts();
  stateA2 = digitalRead(encoderA2);
  stateB2 = digitalRead(encoderB2);
  int change = 0;
  if (stateA2 != lastStateA2) { if (stateA2 != stateB2) change++; else change--; }
  if (stateB2 != lastStateB2) { if (stateB2 == stateA2) change++; else change--; }
  encoderCountRight += change;
  lastStateA2 = stateA2; lastStateB2 = stateB2;
  interrupts();
}

// ================== FUNCIONES DE CONTROL DE MOTORES ==================
// ***** CAMBIO AQUÍ: Lógica PWM invertida *****
// Función para establecer el PWM del motor izquierdo
void setMotorPWMLeft(int pwm) {
  int pwmVal = constrain(abs(pwm), 0, MAX_PWM);
  // AHORA: pwm > 0 (Avance) activa A, desactiva B
  if (pwm > 0) {
    analogWrite(pwmA1, pwmVal); // Pin A activo para avance
    analogWrite(pwmB1, 0);
  }
  // AHORA: pwm < 0 (Retroceso) activa B, desactiva A
  else if (pwm < 0) {
    analogWrite(pwmA1, 0);
    analogWrite(pwmB1, pwmVal); // Pin B activo para retroceso
  } else { // Paro
    analogWrite(pwmA1, 0);
    analogWrite(pwmB1, 0);
  }
}

// ***** CAMBIO AQUÍ: Lógica PWM invertida *****
// Función para establecer el PWM del motor derecho
void setMotorPWMRight(int pwm) {
  int pwmVal = constrain(abs(pwm), 0, MAX_PWM);
  // AHORA: pwm > 0 (Avance) activa A, desactiva B
  if (pwm > 0) {
    analogWrite(pwmA2, pwmVal); // Pin A activo para avance
    analogWrite(pwmB2, 0);
  }
  // AHORA: pwm < 0 (Retroceso) activa B, desactiva A
  else if (pwm < 0) {
    analogWrite(pwmA2, 0);
    analogWrite(pwmB2, pwmVal); // Pin B activo para retroceso
  } else { // Paro
    analogWrite(pwmA2, 0);
    analogWrite(pwmB2, 0);
  }
}
// ********************************************

// Función para detener ambos motores y reiniciar estado PID
void stopMotors() {
  setMotorPWMLeft(0);
  setMotorPWMRight(0);
  desiredSpeedLeft = 0.0;
  desiredSpeedRight = 0.0;
  integralLeft = 0.0;
  integralRight = 0.0;
  lastErrorLeft = 0.0;
  lastErrorRight = 0.0;
  motorsActive = false;
  Serial.println("Motores detenidos y PID reseteado.");
}

// ================== CONFIGURACIÓN (setup) ==================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("\n\n--- Iniciando Control de Velocidad con PID ---");

  pinMode(pwmA1, OUTPUT); pinMode(pwmB1, OUTPUT);
  pinMode(encoderA1, INPUT_PULLUP); pinMode(encoderB1, INPUT_PULLUP);
  pinMode(pwmA2, OUTPUT); pinMode(pwmB2, OUTPUT);
  pinMode(encoderA2, INPUT_PULLUP); pinMode(encoderB2, INPUT_PULLUP);

  setMotorPWMLeft(0); setMotorPWMRight(0);

  noInterrupts();
  lastStateA1 = digitalRead(encoderA1); lastStateB1 = digitalRead(encoderB1);
  lastStateA2 = digitalRead(encoderA2); lastStateB2 = digitalRead(encoderB2);
  interrupts();

  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), handleEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB2), handleEncoderRight, CHANGE);

  Serial.println("Pines y Encoders inicializados.");
  lastControlTime = millis();
}

// ================== LOOP PRINCIPAL ==================
void loop() {
  // Procesar comandos recibidos vía Serial
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    if (inputString.equalsIgnoreCase("s")) {
      stopMotors();
    } else {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        String leftStr = inputString.substring(0, commaIndex);
        String rightStr = inputString.substring(commaIndex + 1);
        float newSpeedLeft = leftStr.toFloat();
        float newSpeedRight = rightStr.toFloat();
        noInterrupts();
        desiredSpeedLeft = newSpeedLeft; desiredSpeedRight = newSpeedRight;
        motorsActive = true;
        interrupts();
        integralLeft = 0.0; integralRight = 0.0;
        lastErrorLeft = 0.0; lastErrorRight = 0.0;
        Serial.print("Nueva velocidad deseada - Izquierda: "); Serial.print(desiredSpeedLeft, 3);
        Serial.print(" m/s, Derecha: "); Serial.print(desiredSpeedRight, 3);
        Serial.println(" m/s. Motores activados.");
      } else {
        Serial.println("Formato incorrecto. Use: <velIzq>,<velDer> (en m/s) o 's' para detener.");
      }
    }
  }

  // Control periódico
  unsigned long currentTime = millis();
  if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (currentTime - lastControlTime) / 1000.0;
    lastControlTime = currentTime;

    // --- Lectura de encoders y cálculo de velocidad medida ---
    long currentEncoderLeft, currentEncoderRight;
    noInterrupts();
    currentEncoderLeft = encoderCountLeft; currentEncoderRight = encoderCountRight;
    interrupts();
    long deltaLeft = currentEncoderLeft - lastEncoderCountLeft;
    long deltaRight = currentEncoderRight - lastEncoderCountRight;
    lastEncoderCountLeft = currentEncoderLeft; lastEncoderCountRight = currentEncoderRight;

    // ***** CAMBIO AQUÍ: Volvemos al cálculo original *****
    // Convertir delta de ticks a velocidad (m/s)
    // Asumimos que delta positivo significa avance físico.
    measuredSpeedLeft = ( (float)deltaLeft * WHEEL_CIRCUMFERENCE ) / (COUNTS_PER_REVOLUTION * dt);
    measuredSpeedRight = ( (float)deltaRight * WHEEL_CIRCUMFERENCE ) / (COUNTS_PER_REVOLUTION * dt);
    // ****************************************************

    // --- Control PID y aplicación de PWM (SOLO SI motorsActive es true) ---
    if (motorsActive) {
      float localDesiredLeft, localDesiredRight;
      noInterrupts();
      localDesiredLeft = desiredSpeedLeft; localDesiredRight = desiredSpeedRight;
      interrupts();

      // PID Izquierdo
      float errorLeft = localDesiredLeft - measuredSpeedLeft;
      integralLeft += errorLeft * dt;
      float derivativeLeft = (dt > 0) ? (errorLeft - lastErrorLeft) / dt : 0.0;
      lastErrorLeft = errorLeft;
      float pidOutputLeft = Kp * errorLeft + Ki * integralLeft + Kd * derivativeLeft;

      // PID Derecho
      float errorRight = localDesiredRight - measuredSpeedRight;
      integralRight += errorRight * dt;
      float derivativeRight = (dt > 0) ? (errorRight - lastErrorRight) / dt : 0.0;
      lastErrorRight = errorRight;
      float pidOutputRight = Kp * errorRight + Ki * integralRight + Kd * derivativeRight;

      // Ajuste mínimo de PWM
      if (localDesiredLeft != 0.0 && pidOutputLeft != 0.0 && abs(pidOutputLeft) < MIN_PWM) {
          pidOutputLeft = (pidOutputLeft > 0) ? MIN_PWM : -MIN_PWM;
      }
      if (localDesiredRight != 0.0 && pidOutputRight != 0.0 && abs(pidOutputRight) < MIN_PWM) {
          pidOutputRight = (pidOutputRight > 0) ? MIN_PWM : -MIN_PWM;
      }

      // Limitar PWM
      pidOutputLeft = constrain(pidOutputLeft, -MAX_PWM, MAX_PWM);
      pidOutputRight = constrain(pidOutputRight, -MAX_PWM, MAX_PWM);

      // Aplicar PWM (con la lógica ya invertida en las funciones setMotorPWM*)
      setMotorPWMLeft((int)pidOutputLeft);
      setMotorPWMRight((int)pidOutputRight);

    } else {
      // Motores inactivos
      setMotorPWMLeft(0);
      setMotorPWMRight(0);
    }

    // Enviar deltas de ticks
    Serial.print(deltaLeft); Serial.print(","); Serial.println(deltaRight);
  }
  delay(1);
}
