#include <Arduino.h>

// ================== DEFINICIONES DE PINES ==================
// Asumiremos que:
 // Motor Izquierdo (Motor 1)
const int pwmA1     = 5;    // Canal PWM para dirección (ej. retroceso)
const int pwmB1     = 4;    // Canal PWM para dirección (ej. avance)
const int encoderA1 = 12;   // Encoder fase A
const int encoderB1 = 13;   // Encoder fase B

// Motor Derecho (Motor 2)
const int pwmA2     = 2;    // Canal PWM para dirección (ej. retroceso)
const int pwmB2     = 3;    // Canal PWM para dirección (ej. avance)
const int encoderA2 = 8;    // Encoder fase A
const int encoderB2 = 9;    // Encoder fase B

// ================== CONSTANTES PARA CONTROL ==================
#define CONTROL_INTERVAL_MS 50  // Intervalo de control en ms

// Parámetros del encoder y del robot
const float COUNTS_PER_REVOLUTION = 4532.0;   // Pulsos por revolución
const float WHEEL_RADIUS = 0.035;                // Radio de la rueda en metros (ej. 3.5 cm)
const float WHEEL_CIRCUMFERENCE = 2.0 * PI * WHEEL_RADIUS; // Circunferencia en metros

// Parámetros del PID (ajusta estos valores en función de tus pruebas)
float Kp = 100.0;
float Ki = 50.0;
float Kd = 10.0;

// Límites de la señal PWM
const int MIN_PWM = 98;
const int MAX_PWM = 250;

// ================== VARIABLES GLOBALES PARA CONTROL DE VELOCIDAD ==================
// Velocidad deseada en m/s para cada rueda (se actualizan vía Serial)
volatile float desiredSpeedLeft = 0.0;
volatile float desiredSpeedRight = 0.0;

// Variables para almacenar la velocidad medida (m/s)
float measuredSpeedLeft = 0.0;
float measuredSpeedRight = 0.0;

// Variables para el lazo PID de cada motor
float integralLeft = 0.0, lastErrorLeft = 0.0;
float integralRight = 0.0, lastErrorRight = 0.0;

// Variables para el control de encoders (acumulados)
volatile long encoderCountLeft = 0;   // Motor izquierdo (Motor 1)
volatile long encoderCountRight = 0;  // Motor derecho (Motor 2)

// Para calcular el delta de ticks en cada ciclo
volatile long lastEncoderCountLeft = 0;
volatile long lastEncoderCountRight = 0;

// Tiempo de control
unsigned long lastControlTime = 0;

// ================== VARIABLES PARA LAS INTERRUPCIONES DE LOS ENCODERS ==================
// Motor Izquierdo
volatile byte lastStateA1 = 0;
volatile byte lastStateB1 = 0;
volatile byte stateA1 = 0;
volatile byte stateB1 = 0;

// Motor Derecho
volatile byte lastStateA2 = 0;
volatile byte lastStateB2 = 0;
volatile byte stateA2 = 0;
volatile byte stateB2 = 0;

// ================== ISR DE LOS ENCODERS ==================
// ISR para el motor izquierdo (Motor 1)
IRAM_ATTR void handleEncoderLeft() {
  noInterrupts();
  stateA1 = digitalRead(encoderA1);
  stateB1 = digitalRead(encoderB1);
  if (stateA1 != lastStateA1) {
    if (stateA1 != stateB1) {
      encoderCountLeft++;
    } else {
      encoderCountLeft--;
    }
  }
  if (stateB1 != lastStateB1) {
    if (stateB1 == stateA1) {
      encoderCountLeft++;
    } else {
      encoderCountLeft--;
    }
  }
  lastStateA1 = stateA1;
  lastStateB1 = stateB1;
  interrupts();
}

// ISR para el motor derecho (Motor 2)
IRAM_ATTR void handleEncoderRight() {
  noInterrupts();
  stateA2 = digitalRead(encoderA2);
  stateB2 = digitalRead(encoderB2);
  if (stateA2 != lastStateA2) {
    if (stateA2 != stateB2) {
      encoderCountRight++;
    } else {
      encoderCountRight--;
    }
  }
  if (stateB2 != lastStateB2) {
    if (stateB2 == stateA2) {
      encoderCountRight++;
    } else {
      encoderCountRight--;
    }
  }
  lastStateA2 = stateA2;
  lastStateB2 = stateB2;
  interrupts();
}

// ================== FUNCIONES DE CONTROL DE MOTORES ==================
// Función para establecer el PWM del motor izquierdo según la señal (positivo = avance, negativo = retroceso)
void setMotorPWMLeft(int pwm) {
  int pwmVal = constrain(abs(pwm), 0, MAX_PWM);
  if (pwm > 0) {
    // Avance: canal B activa, A en 0
    analogWrite(pwmA1, 0);
    analogWrite(pwmB1, pwmVal);
  } else if (pwm < 0) {
    // Retroceso: canal A activa, B en 0
    analogWrite(pwmA1, pwmVal);
    analogWrite(pwmB1, 0);
  } else {
    // Paro
    analogWrite(pwmA1, 0);
    analogWrite(pwmB1, 0);
  }
}

// Función para establecer el PWM del motor derecho según la señal
void setMotorPWMRight(int pwm) {
  int pwmVal = constrain(abs(pwm), 0, MAX_PWM);
  if (pwm > 0) {
    analogWrite(pwmA2, 0);
    analogWrite(pwmB2, pwmVal);
  } else if (pwm < 0) {
    analogWrite(pwmA2, pwmVal);
    analogWrite(pwmB2, 0);
  } else {
    analogWrite(pwmA2, 0);
    analogWrite(pwmB2, 0);
  }
}

// Función para detener ambos motores y reiniciar integrales
void stopMotors() {
  setMotorPWMLeft(0);
  setMotorPWMRight(0);
  desiredSpeedLeft = 0.0;
  desiredSpeedRight = 0.0;
  integralLeft = 0;
  integralRight = 0;
  Serial.println("Motores detenidos.");
}

// ================== CONFIGURACIÓN (setup) ==================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n--- Iniciando Control de Velocidad con PID ---");

  // Configuración de pines para el Motor Izquierdo
  pinMode(pwmA1, OUTPUT);
  pinMode(pwmB1, OUTPUT);
  pinMode(encoderA1, INPUT_PULLUP);
  pinMode(encoderB1, INPUT_PULLUP);

  // Configuración de pines para el Motor Derecho
  pinMode(pwmA2, OUTPUT);
  pinMode(pwmB2, OUTPUT);
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);

  // Inicializar salidas PWM a 0
  analogWrite(pwmA1, 0);
  analogWrite(pwmB1, 0);
  analogWrite(pwmA2, 0);
  analogWrite(pwmB2, 0);

  // Inicializar estados de los encoders
  lastStateA1 = digitalRead(encoderA1);
  lastStateB1 = digitalRead(encoderB1);
  lastStateA2 = digitalRead(encoderA2);
  lastStateB2 = digitalRead(encoderB2);

  // Adjuntar interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), handleEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB2), handleEncoderRight, CHANGE);

  Serial.println("Encoders inicializados.");
  lastControlTime = millis();
}

// ================== LOOP PRINCIPAL ==================
void loop() {
  // Procesar comandos recibidos vía Serial (velocidades deseadas en m/s)
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    
    // Comando especial para detener
    if (inputString.equalsIgnoreCase("s")) {
      stopMotors();
    }
    else {
      // Se espera el formato "0.2,0.2" (valores separados por coma)
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        String leftStr = inputString.substring(0, commaIndex);
        String rightStr = inputString.substring(commaIndex + 1);
        float newSpeedLeft = leftStr.toFloat();
        float newSpeedRight = rightStr.toFloat();
        desiredSpeedLeft = newSpeedLeft;
        desiredSpeedRight = newSpeedRight;
        Serial.print("Nueva velocidad deseada - Izquierda: ");
        Serial.print(desiredSpeedLeft, 3);
        Serial.print(" m/s, Derecha: ");
        Serial.print(desiredSpeedRight, 3);
        Serial.println(" m/s");
      } else {
        Serial.println("Formato incorrecto. Use: <velIzq>,<velDer> (en m/s)");
      }
    }
  }

  // Control periódico cada CONTROL_INTERVAL_MS milisegundos
  unsigned long currentTime = millis();
  if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (currentTime - lastControlTime) / 1000.0;  // tiempo en segundos
    lastControlTime = currentTime;
    
    // --- Cálculo de velocidad medida a partir de los encoders ---
    // Guardamos los contadores actuales y calculamos el delta
    long currentEncoderLeft = encoderCountLeft;
    long currentEncoderRight = encoderCountRight;
    long deltaLeft = currentEncoderLeft - lastEncoderCountLeft;
    long deltaRight = currentEncoderRight - lastEncoderCountRight;
    lastEncoderCountLeft = currentEncoderLeft;
    lastEncoderCountRight = currentEncoderRight;
    
    // Convertir delta de ticks a velocidad (m/s)
    // revoluciones = delta / COUNTS_PER_REVOLUTION, distancia = revoluciones * circunferencia
    measuredSpeedLeft = ( (float)deltaLeft * WHEEL_CIRCUMFERENCE ) / (COUNTS_PER_REVOLUTION * dt);
    measuredSpeedRight = ( (float)deltaRight * WHEEL_CIRCUMFERENCE ) / (COUNTS_PER_REVOLUTION * dt);
    
    // --- Control PID para cada motor ---
    // Motor Izquierdo
    float errorLeft = desiredSpeedLeft - measuredSpeedLeft;
    integralLeft += errorLeft * dt;
    float derivativeLeft = (errorLeft - lastErrorLeft) / dt;
    lastErrorLeft = errorLeft;
    float pidOutputLeft = Kp * errorLeft + Ki * integralLeft + Kd * derivativeLeft;
    
    // Motor Derecho
    float errorRight = desiredSpeedRight - measuredSpeedRight;
    integralRight += errorRight * dt;
    float derivativeRight = (errorRight - lastErrorRight) / dt;
    lastErrorRight = errorRight;
    float pidOutputRight = Kp * errorRight + Ki * integralRight + Kd * derivativeRight;
    
    // Ajuste mínimo de PWM: si la salida es pequeña pero no cero, forzar el mínimo
    if ( (pidOutputLeft != 0) && (abs(pidOutputLeft) < MIN_PWM) ) {
      pidOutputLeft = (pidOutputLeft > 0) ? MIN_PWM : -MIN_PWM;
    }
    if ( (pidOutputRight != 0) && (abs(pidOutputRight) < MIN_PWM) ) {
      pidOutputRight = (pidOutputRight > 0) ? MIN_PWM : -MIN_PWM;
    }
    
    // Limitar la señal PWM
    pidOutputLeft = constrain(pidOutputLeft, -MAX_PWM, MAX_PWM);
    pidOutputRight = constrain(pidOutputRight, -MAX_PWM, MAX_PWM);
    
    // Aplicar la señal PWM a los motores
    setMotorPWMLeft((int)pidOutputLeft);
    setMotorPWMRight((int)pidOutputRight);
    
    // Enviar odometría cruda: enviar los deltas de ticks en este intervalo
    Serial.print(deltaLeft);
    Serial.print(",");
    Serial.println(deltaRight);
  }
  
  // Pequeño delay para no saturar el loop
  delay(1);
}
