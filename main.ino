#include <Servo.h>

// Pinagem
#define RIGHT_MOTOR_PIN 10 // motor direito
#define LEFT_MOTOR_PIN 9 // motor esquerdo
#define RIGHT_LINE_SENSOR_PIN 2 // sensor de linha direito
#define LEFT_LINE_SENSOR_PIN 3 // sensor de linha esquerdo
#define RIGHT_OBSTACLE_SENSOR_PIN 17 // sharp direito
#define LEFT_OBSTACLE_SENSOR_PIN 18 // sharp esquerdo

#define ARENA_DIAMETER 77

Servo right_motor;
Servo left_motor;

void setup() {
  right_motor.attach(RIGHT_MOTOR_PIN); 
  left_motor.attach(LEFT_MOTOR_PIN); 
  right_motor.write(90); 
  left_motor.write(90); 
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT); 
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_OBSTACLE_SENSOR_PIN, INPUT); 
  pinMode(LEFT_OBSTACLE_SENSOR_PIN, INPUT); 
  Serial.begin(9600); 
}

void frente(int speed) {
  Serial.println('frente...'); 
  right_motor.write(90 + speed); 
  left_motor.write(90 + speed);
}

void tras(int speed) {
  Serial.println('tras...'); 
  right_motor.write(90 - speed); 
  left_motor.write(90 - speed); 
}

void direita(int speed) {
  Serial.println('direita...'); 
  right_motor.write(90 - speed); 
  left_motor.write(90 + speed);
}

void esquerda(int speed) {
  Serial.println('esquerda...'); 
  right_motor.write(90 + speed); 
  left_motor.write(90 - speed); 
}

int getDistance(int sensor_pin) {
  float voltage = analogRead(sensor_pin) * (5.0 / 1023.0); // Convert analog value to voltage
  float distanceCM = 27.86 * pow(voltage, -1.15);     // Calculate distance in cm
  return distanceCM; 
}

void sens_obs() {
  int sod = getDistance(RIGHT_OBSTACLE_SENSOR_PIN); 
  int soe = getDistance(LEFT_OBSTACLE_SENSOR_PIN); 

  if (sod > ARENA_DIAMETER && soe > ARENA_DIAMETER) {
    frente(45); 
  }

  if (sod > ARENA_DIAMETER && soe <= ARENA_DIAMETER) {
    esquerda(90); 
  }

  if (sod <= ARENA_DIAMETER && soe > ARENA_DIAMETER) {
    direita(90); 
  }

  if (sod <= ARENA_DIAMETER && soe <= ARENA_DIAMETER) {
    frente(90); 
  }
}

void loop() {
  sens_obs(); 

  if(digitalRead(LEFT_LINE_SENSOR_PIN) == LOW) {
    tras(90); 
    delay(100); 
    direita(90); 
    delay(150); 
  }

  if(digitalRead(RIGHT_LINE_SENSOR_PIN) == LOW) {
    tras(90); 
    delay(100); 
    esquerda(90); 
    delay(150); 
  }

}
