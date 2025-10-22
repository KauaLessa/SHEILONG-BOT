#include <SparkFun_TB6612.h>

const int ARENA_DIAMETER = 77; 

// Pinagem

// Motor direito
const int RIGHT_MOTOR_AI1_PIN = 4; 
const int RIGHT_MOTOR_AI2_PIN = 7; 
const int RIGHT_MOTOR_PWM_PIN = 6; 

// Motor esquerdo
const int LEFT_MOTOR_BI1_PIN = 8; 
const int LEFT_MOTOR_BI2_PIN = 9; 
const int LEFT_MOTOR_PWM_PIN = 5; 

// Sensores de linha
const int RIGHT_LINE_SENSOR_PIN = 2; 
const int LEFT_LINE_SENSOR_PIN = 3; 

// Sensores de obstáculo
const int RIGHT_OBS_SENSOR_PIN = A4; 
const int LEFT_OBS_SENSOR_PIN = A3; 

Motor right_motor = Motor(
  RIGHT_MOTOR_AI1_PIN, 
  RIGHT_MOTOR_AI2_PIN, 
  RIGHT_MOTOR_PWM_PIN,
  1,
  A7
);

Motor left_motor = Motor(
  LEFT_MOTOR_BI1_PIN,
  LEFT_MOTOR_BI2_PIN,
  LEFT_MOTOR_PWM_PIN,
  1,
  A7
);

void setup() {
  Serial.begin(9600);
  delay(5000); 
}

int getDistance(int sensor_pin) {
  float voltage = analogRead(sensor_pin) * (5.0 / 1023.0); // Convert analog value to voltage
  float distanceCM = 27.86 * pow(voltage, -1.15);     // Calculate distance in cm
  return distanceCM; 
}

void turnLeft(int speed) {
  right_motor.drive(speed); 
  left_motor.drive(-speed); 
}

void turnRight(int speed) {
  right_motor.drive(-speed); 
  left_motor.drive(speed); 
}

void sens_obs() {
  int sod = getDistance(RIGHT_OBS_SENSOR_PIN); 
  int soe = getDistance(LEFT_OBS_SENSOR_PIN); 

  if (sod > ARENA_DIAMETER && soe > ARENA_DIAMETER) {
    Serial.println("Frente lento...");
    forward(right_motor, left_motor, 128); 
  }

  if (sod > ARENA_DIAMETER && soe <= ARENA_DIAMETER) {
    Serial.println("Esquerda...");
    turnLeft(255); 
  }

  if (sod <= ARENA_DIAMETER && soe > ARENA_DIAMETER) {
    Serial.println("Direita...");
    turnRight(255); 
  }

  if (sod <= ARENA_DIAMETER && soe <= ARENA_DIAMETER) {
    Serial.println("Frente rapido...");
    forward(right_motor, left_motor, 255); 
  }
}

void loop() {
  sens_obs(); 

  if(digitalRead(LEFT_LINE_SENSOR_PIN) == LOW) {
    back(right_motor, left_motor, 255); 
    Serial.println("tras..."); 
    delay(200); 
    turnRight(255); 
    Serial.println("Direita..."); 
    delay(250); 
  }

  if(digitalRead(RIGHT_LINE_SENSOR_PIN) == LOW) {
    back(right_motor, left_motor, 255); 
    Serial.println("tras..."); 
    delay(200); 
    turnLeft(255); 
    Serial.println("Esquerda..."); 
    delay(250); 
  }

}
